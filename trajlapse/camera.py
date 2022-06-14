#!/usr/bin/env python3
import os
from collections import namedtuple, OrderedDict
import time
import threading
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

#try:
#    import colors as _colors
#except ImportError:
#    logger.warning("Colors module not available, using slow Python implementation")
#    colors = None
#else:
#    colors = _colors.Flatfield("flatfield.txt")
#    sRGB = _colors.SRGB()

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


class CameraSimple(threading.Thread):
    "A simple camera class which continuously saves images"

    def __init__(self, resolution=(4056, 3040), framerate=1, sensor_mode=3,
                 avg_ev=21, avg_wb=31, histo_ev=None,
                 wb_red=None, wb_blue=None,
                 quit_event=None, queue=None,
                 folder="/tmp",
                 ):
        """This thread handles the camera: simple camera saving data to 
        
        """
        self.avg_ev = avg_ev
        self.avg_wb = avg_wb

        self.histo_ev = histo_ev or []
        self.wb_red = wb_red or []
        self.wb_blue = wb_blue or []

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

    def __del__(self):
        self.camera = self.stream = None

    def quit(self, *arg, **kwarg):
        "quit the main loop and end the thread"
        self.quit_event.set()

    def pause(self, wait=True):
        "pause the recording, wait for the current value to be acquired"
        self.lock.acquire(blocking=wait)

    def resume(self):
        "resume the recording"
        self.lock.release

    def shoot(self):
        self.record_event.set()

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
        #stream = io
        while time.time() < end:
            try:
                self.collect_exposure()
            except ZeroDivisionError:
                end = time.time() + delay
                continue
            time.sleep(1.0 / self.camera.framerate)
        # keep only last value
        self.wb_red = wb_red + self.wb_red[-1:]
        self.wb_blue = wb_blue + self.wb_blue[-1:]
        self.histo_ev = histo_ev + self.histo_ev[-1:]

    def run(self):
        "main thread activity"
        stream = io.BytesIO()
        #stream = "/tmp/trajlsapse_last.jpg"
        self.set_exposure_auto()
        while not self.quit_event.is_set():
            if self.record_event.is_set():
                # record !
                self.set_exposure_fixed()
                with self.lock:
                    before = time.time()
                    filename = get_isotime(before) + ".jpg"
                    fullname = os.path.join(self.folder, filename)
                    self.camera.capture(fullname, format="jpeg", thumbnail=None)
                    after = time.time()
                metadata = self.get_metadata()
                self.set_exposure_auto()
                metadata["filename"] = filename
                metadata["camera_start"] = before
                metadata["camera_stop"] = after
                self.queue.put(metadata)
                self.record_event.clear()

                self.camera.capture(stream, format="jpeg", 
                                    thumbnail=None,
                                    use_video_port=True 
                                    )
                stream.truncate()
                stream.seek(0)
            else:
                #acquires dummy image for exposure calibration
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
            #except ZeroDivisionError:
                #continue
        self.camera.close()

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
        #logger.info("Update speed: %s %s", speed, framerate)

        if speed > framerate:
            self.camera.shutter_speed = int(1e6 / framerate / speed)
            self.camera.iso = 100
        elif speed > framerate * 2:
            self.camera.shutter_speed = int(2e6 / framerate / speed)
            self.camera.iso = 200
        elif speed > framerate * 4:
            self.camera.shutter_speed = int(4e6 / framerate / speed)
            self.camera.iso = 400
        else:
            self.camera.shutter_speed = min(int(8e6 / framerate / speed), int(1e6 / framerate))
            self.camera.iso = 800

    def set_exposure_auto(self):
        self.camera.shutter_speed = 0
        self.camera.iso = 0
        self.camera.awb_mode = "auto"  # alternative: off
        self.camera.meter_mode = 'average'  # 'backlit' #"average" ?
        self.camera.exposure_mode = "auto" #"nightpreview"  # auto"
        self.camera.start_preview()

    def set_exposure_fixed(self):
        self.camera.stop_preview()
        self.update_expo()
        self.camera.awb_mode = "off"
        self.camera.exposure_mode = "off"

    def collect_exposure(self, filename=None):
        if filename:
            ev = self.get_exposure_fn(filename)
        else:
            ev = self.get_exposure()
        self.histo_ev.append(ev)
        rg, bg = self.camera.awb_gains
        self.wb_red.append(1.0 if rg == 0 else float(rg))
        self.wb_blue.append(1.0 if bg == 0 else float(bg))
        #return ev,rg,bg

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
        time = exif["Exif.Photo.ExposureTime"].toFloat()
        speed = exif["Exif.Photo.ShutterSpeedValue"].toFloat()
        Ev =  exif["Exif.Photo.BrightnessValue"].toFloat()
        ev = lens.calc_EV(speed, iso=iso)
        if isinstance(filename, bytes):
            filename="bytes"
        logger.debug(f"filename {filename} Ev: {Ev} {ev}, ISO: {iso}, speed: {speed}, time: {time}")
        return ev
