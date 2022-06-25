#!/usr/bin/env python3
import os
from collections import namedtuple, OrderedDict
import time
import threading
from multiprocessing import Process, Queue as MpQueue
import io
from queue import Queue
import logging
import numpy
from picamera import PiCamera
from scipy.signal import savgol_coeffs
from .exposure import lens
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("camera")
import signal
import exiv2


def analyzer(shape, qin, qout):
    "Simple analyzer process"
    from trajlapse.analysis  import Analyzer
    a = Analyzer(shape)
    metadata = qin.get()
    while metadata is not None:
        fname = metadata.get("filename")
        if fname and os.path.exists(fname):
            res = a.process(fname)
            metadata.update(res)
            qout.put(metadata)
            os.unlink(fname)
        metadata = qin.get()


ExpoRedBlue = namedtuple("ExpoRedBlue", ("ev", "red", "blue"))
GainRedBlue = namedtuple("GainRedBlue", ("red", "blue"))


def get_isotime(force_time=None):
    """
    :param force_time: enforce a given time (current by default)
    :type force_time: float
    :return: the current time as an ISO8601 string
    :rtype: string
    """
    if force_time is None:
        force_time = time.time()
    localtime = time.localtime(force_time)
    # gmtime = time.gmtime(force_time)
    # tz_h = localtime.tm_hour - gmtime.tm_hour
    # tz_m = localtime.tm_min - gmtime.tm_min
    # return f"{time.strftime('%Y-%m-%dT%H:%M:%S', localtime)}{tz_h:+03d}:{tz_m:02d}"
    return time.strftime('%Y-%m-%d-%Hh%Mm%Ss', localtime)


class SavGol(object):
    "Class for Savitsky-Golay filtering"

    def __init__(self, order=2):
        "select the order of the filter"
        self.order = order
        self.cache = {}  # len, filter

    def __call__(self, lst):
        "filter a list. the last having the more weight"
        l = len(lst)
        if l % 2 == 0:
            lst = numpy.array(lst[1:])
            l -= 1
        else:
            lst = numpy.array(lst)
        if len(lst) <= self.order:
            return lst[-1]
        if l not in self.cache:
            self.cache[l] = savgol_coeffs(l, self.order, pos=0)
        return numpy.dot(lst, self.cache[l])


savgol0 = SavGol(0)
savgol1 = SavGol(1)


class Camera(threading.Thread):
    "A camera class which continuously saves images, analysis performed in separate process"

    def __init__(self, resolution=(4056, 3040), framerate=1, sensor_mode=3,
                 avg_ev=21, avg_wb=31, histo_ev=None,
                 wb_red=None, wb_blue=None,
                 quit_event=None, queue=None,
                 folder="/tmp",
                 ):
        """This thread handles the camera: simple camera saving data to a file
        
        """
        self.analysis_folder = None
        self.avg_ev = avg_ev
        self.avg_wb = avg_wb
        self.histo_ev = histo_ev or []
        self.wb_red = wb_red or []
        self.wb_blue = wb_blue or []
        self.exposure_mode = "auto" # can be night-preview
        threading.Thread.__init__(self, name="Camera")
        signal.signal(signal.SIGINT, self.quit)
        self.quit_event = quit_event or threading.Event()
        self.record_event = threading.Event()
        self.folder = folder
        self.queue = queue or Queue()
        self.last_index = -1
        self.last_subindex = -1
        self.lock = threading.Semaphore()
        self.camera = PiCamera(resolution=resolution, framerate=framerate, sensor_mode=sensor_mode)
        self.nproc = 2
        self.max_analysis = 10
        self.analysis_queue_in = self.analysis_queue_out = self.analyser_pool = None
        self.setup_analyzer_pool()

    def __del__(self):
        self.camera = self.stream = None
        self.analysis_queue_in = self.analysis_queue_out = self.analyser_pool = None

    def quit(self, *arg, **kwarg):
        "quit the main loop and end the thread"
        self.quit_event.set()
        self.camera.close()
        if self.analyser_pool:
            for p in self.analyser_pool:
                self.analysis_queue_in.put(None)
            for p in self.analyser_pool:
                p.join()
        self.analyser_pool = []

    def pause(self, wait=True):
        "pause the recording, wait for the current value to be acquired"
        self.lock.acquire(blocking=wait)

    def resume(self):
        "resume the recording"
        self.lock.release

    def shoot(self):
        self.record_event.set()

    def setup_analyzer_pool(self):
        self.analysis_folder = "/tmp/trajlapse"
        if not os.path.isdir(self.analysis_folder):
            os.makedirs(self.analysis_folder)
        for f in os.listdir(self.analysis_folder):
            os.unlink(os.path.join(self.analysis_folder, f))

        self.analysis_queue_in = MpQueue()
        self.analysis_queue_out = MpQueue()
        self.analyser_pool = [Process(target=analyzer, args=(self.camera.resolution,
                                                             self.analysis_queue_in,
                                                             self.analysis_queue_out))
                                for i in range(self.nproc)]
        for p in self.analyser_pool:
            p.start()

    def get_config(self):
        config = OrderedDict([("resolution", tuple(self.camera.resolution)),
                              ("framerate", float(self.camera.framerate)),
                              ("sensor_mode", self.camera.sensor_mode),
                              ("avg_ev", self.avg_ev),
                              ("avg_wb", self.avg_wb),
                              ("hist_ev", self.histo_ev),
                              ("wb_red", self.wb_red),
                              ("wb_blue", self.wb_blue),
                              ("folder", self.folder)
                              ]
        )
        return config

    def set_config(self, dico):
        self.camera.resolution = dico.get("resolution", self.camera.resolution)
        self.camera.framerate = dico.get("framerate", self.camera.framerate)
        self.camera.sensor_mode = dico.get("sensor_mode", self.camera.sensor_mode)
        self.wb_red = dico.get("wb_red", self.wb_red)
        self.wb_blue = dico.get("wb_blue", self.wb_blue)
        self.histo_ev = dico.get("histo_ev", self.histo_ev)
        self.avg_ev = dico.get("avg_ev", self.avg_ev)
        self.avg_wb = dico.get("avg_wb", self.avg_wb)
        self.folder = dico.get("folder", self.folder)

    def warm_up(self, delay=10):
        "warm up the camera"
        end = time.time() + delay
        logger.info(f"warming up the camera for {delay}s")
        self.set_exposure_auto()
        #backup
        wb_red = self.wb_red[:]
        wb_blue = self.wb_blue[:]
        histo_ev = self.histo_ev[:]
        self.wb_red = []
        self.wb_blue = []
        self.histo_ev = []
        stream = io.BytesIO()
        while time.time() < end:
            self.camera.capture(stream, format="jpeg", 
                                thumbnail=None,
                                #use_video_port=False
                                )
            stream.truncate()
            stream.seek(0)
            #    time.sleep(1.0 / self.camera.framerate)
            #try:
            self.collect_exposure(stream.read())
            stream.seek(0)
            stream.truncate()

        self.wb_red = wb_red + self.wb_red
        self.wb_blue = wb_blue + self.wb_blue
        self.histo_ev = histo_ev + self.histo_ev

    def run(self):
        "main thread activity"
        idx = 0
        self.set_exposure_fixed()
        while not self.quit_event.is_set():
            self.set_exposure()
            if self.record_event.is_set():
                # record !
                # self.set_exposure_fixed()
                with self.lock:
                    before = time.time()
                    filename = get_isotime(before) + ".jpg"
                    fullname = os.path.join(self.folder, filename)
                    self.camera.capture(fullname, format="jpeg", thumbnail=None)
                    after = time.time()
                metadata = self.get_metadata()
                # self.set_exposure_auto()
                metadata["filename"] = filename
                metadata["camera_start"] = before
                metadata["camera_stop"] = after
                self.queue.put(metadata)
                self.record_event.clear()
            else:
                #acquires dummy image for exposure calibration
                if len(os.listdir(self.analysis_folder)) < self.max_analysis:
                    idx += 1
                    filename = os.path.join(self.analysis_folder, f"temp_{idx:04d}.jpg")
                    self.camera.capture(filename, format="jpeg",
                                        thumbnail=None)
                    metadata = self.get_metadata()
                    metadata["filename"] = filename
                    self.analysis_queue_in.put(metadata)
                else:
                    logger.warning("Too many file awaiting for processing")
            time.sleep(0.1 / self.camera.framerate)
        self.quit()

    def get_metadata(self):
        metadata = {"iso": float(self.camera.iso),
                    "analog_gain": float(self.camera.analog_gain),
                    "awb_gains": [float(i) for i in self.camera.awb_gains],
                    "digital_gain": float(self.camera.digital_gain),
                    "exposure_compensation": float(self.camera.exposure_compensation),
                    "exposure_speed": float(self.camera.exposure_speed),
                    "exposure_mode": self.camera.exposure_mode,
                    "framerate": float(self.camera.framerate),
                    "revision": self.camera.revision,
                    "shutter_speed": float(self.camera.shutter_speed),
                    "aperture": lens.aperture,
                    "focal_length": lens.focal,
                    "resolution": self.camera.resolution}
        if metadata['revision'] == "imx219":
            metadata['iso_calc'] = 54.347826086956516 * metadata["analog_gain"] * metadata["digital_gain"]
        if metadata['revision'] == "imx477":
            metadata['iso_calc'] = 40 * metadata["analog_gain"] * metadata["digital_gain"]
        else:
            metadata['iso_calc'] = 100.0 * metadata["analog_gain"] * metadata["digital_gain"]
        return metadata

    def update_expo(self):
        """This method updates the white balance, exposure time and gain
        according to the history
        """
        # return #disabled for now
        if len(self.wb_red) * len(self.wb_blue) == 0:
            return
        if len(self.wb_red) > self.avg_wb:
            self.wb_red = self.wb_red[-self.avg_wb:]
            self.wb_blue = self.wb_blue[-self.avg_wb:]
        if len(self.histo_ev) > self.avg_ev:
            self.histo_ev = self.histo_ev[-self.avg_ev:]
        self.camera.awb_gains = (savgol0(self.wb_red),
                                 savgol0(self.wb_blue))
        ev = savgol1(self.histo_ev)
        speed = lens.calc_speed(ev)
        framerate = float(self.camera.framerate)
        logger.debug(f"Set Exposure to {ev:.3f}  speed: {speed:.3f} {framerate:.3f}")

        if speed > framerate:
            self.camera.iso = 100
            self.camera.shutter_speed = int(1e6 / speed)
        elif speed > framerate * 2:
            self.camera.iso = 200
            self.camera.shutter_speed = int(5e5 / speed)
        elif speed > framerate * 4:
            self.camera.iso = 400
            self.camera.shutter_speed = int(2.5e5 / speed)
        else:
            self.camera.iso = 800
            self.camera.shutter_speed = min(int(1.25e5 / speed), int(1e6 / framerate))

    def set_exposure_auto(self):
        self.camera.shutter_speed = 0
        self.camera.iso = 0
        self.camera.awb_mode = "auto"  # alternative: off
        self.camera.meter_mode = "average"
        self.camera.exposure_mode = self.exposure_mode
        self.still_stats = True
        #self.camera.start_preview()

    def set_exposure_fixed(self):
        #self.camera.stop_preview()
        self.still_stats = False
        self.update_expo()
        self.camera.awb_mode = "off"
        self.camera.exposure_mode = "off"

    def collect_exposure(self, filename=None):
        ev = self.get_exposure_fn(filename) if filename else self.get_exposure()
        self.histo_ev.append(ev)
        rg, bg = self.camera.awb_gains
        self.wb_red.append(1.0 if rg == 0 else float(rg))
        self.wb_blue.append(1.0 if bg == 0 else float(bg))
        #return ev,rg,bg

    def set_exposure(self):
        "Empty queue with processed info and update history. finally set iso and wb"
        while not self.analysis_queue_out.empty():
            result = self.analysis_queue_out.get()
            print(result)

    def get_exposure(self):
        ag = float(self.camera.analog_gain)
        dg = float(self.camera.digital_gain)
        es = self.camera.exposure_speed
        gain = float(ag * dg)
        speed = 1e6 / es
        ev = lens.calc_EV(speed, gain)
        logger.debug(f"ag: {ag} dg: {dg} es: {es} speed {speed} ev: {ev}")
        return ev

    def get_exposure_fn(self, filename):
        image = exiv2.ImageFactory.open(filename)
        image.readMetadata()
        exif = image.exifData()
        iso = exif["Exif.Photo.ISOSpeedRatings"].toLong()
        #time = exif["Exif.Photo.ExposureTime"].toFloat()
        speed = exif["Exif.Photo.ShutterSpeedValue"].toFloat()
        Ev =  exif["Exif.Photo.BrightnessValue"].toFloat()
        ev = lens.calc_EV(speed, iso=iso)
        if isinstance(filename, bytes):
            filename="bytes"
        logger.debug(f"filename {filename} Ev: {Ev:.3f} {ev:.3f}, ISO: {iso}, speed: {speed:.3f}")
        if speed < 4*self.camera.framerate and self.exposure_mode == "auto":
            self.camera.exposure_mode = self.exposure_mode = "nightpreview"  # auto"
            logger.info(f"set exposure to {self.exposure_mode}")
        elif speed > 20*self.camera.framerate and self.exposure_mode == "nightpreview":
            self.camera.exposure_mode = self.exposure_mode = "auto"  # auto"
            logger.info(f"set exposure to {self.exposure_mode}")
        return ev
