#!/usr/bin/env python3
import os
from math import log
from collections import namedtuple, OrderedDict
import time
import threading
import json
import gc
try:
    from Queue import Queue
except ImportError:
    from queue import Queue
import io
import logging
from fractions import Fraction
import numpy
from PIL import Image
# import pyexiv2
from picamera import PiCamera
from scipy.signal import convolve, gaussian, savgol_coeffs
from .exposure import lens
logger = logging.getLogger("camera")
import signal
try:
    import colors as _colors
except ImportError:
    logger.warning("Colors module not available, using slow Python implementation")
    colors = None
else:
    colors = _colors.Flatfield("flatfield.txt")
    sRGB = _colors.SRGB()
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
    gmtime = time.gmtime(force_time)
    tz_h = localtime.tm_hour - gmtime.tm_hour
    tz_m = localtime.tm_min - gmtime.tm_min
    return f"{time.strftime('%Y-%m-%dT%H:%M:%S', localtime)}{tz_h:+03d}:{tz_m:02d}"


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
        if len(lst) < self.order:
            return lst[-1]
        if l not in self.cache:
            self.cache[l] = savgol_coeffs(l, self.order, pos=0)
        return numpy.dot(lst, self.cache[l])


savgol0 = SavGol(0)
savgol1 = SavGol(1)


class Frame(object):
    """This class holds one image"""
    BINNING = 1
    INDEX = 0
    semclass = threading.Semaphore()
            # YUV conversion matrix from ITU-R BT.601 version (SDTV)
            #                  Y       U       V
    YUV2RGB = numpy.array([[1.164, 0.000, 1.596],  # R
                           [1.164, -0.392, -0.813],  # G
                           [1.164, 2.017, 0.000]]).T  # B

    def __init__(self, data):
        "Constructor"
        self.timestamp = time.time()
        with self.semclass:
            self.index = self.INDEX
            self.__class__.INDEX += 1
        self.data = data
        self.camera_meta = {}
        self.gravity = None
        self.position = None
        self.servo_status = None
        self.sem = threading.Semaphore()
        self._yuv = None
        self._rgb = None
        self._histograms = None

    def __repr__(self):
        return "Frame #%04i" % self.index

    def get_date_time(self):
        return time.strftime("%Y-%m-%d-%Hh%Mm%Ss", time.localtime(self.timestamp))

    @property
    def yuv(self):
        """Retrieve the YUV array, binned 2x2 or not depending on the BINNING class attribute"""
        if self._yuv is None:
            with self.sem:
                if self._yuv is None:
                    resolution = self.camera_meta.get("resolution", (640, 480))
                    if colors:
                        yuv = _colors.yuv420_to_yuv(self.data, resolution)[0]
                    else:
                        width, height = resolution
                        fwidth = (width + 31) & ~(31)
                        fheight = (height + 15) & ~(15)
                        ylen = fwidth * fheight
                        uvlen = ylen // 4
                        ary = numpy.frombuffer(self.data, dtype=numpy.uint8)
                        if self.BINNING == 2:
                            Y_full = (ary[:ylen]).astype(numpy.int16)
                            Y_full.shape = (fheight, fwidth)
                            Y = (Y_full[::2,::2] + Y_full[::2, 1::2] + Y_full[1::2,::2] + Y_full[1::2, 1::2]) // 4
                            U = ary[ylen:-uvlen].reshape((fheight // 2, fwidth // 2))
                            V = ary[-uvlen:].reshape((fheight // 2, fwidth // 2))
                            yuv = numpy.dstack((Y.astype(numpy.uint8), U, V))[:height // 2,:width // 2,:]
                        else:
                            # Reshape the values into two dimensions, and double the size of the
                            # U and V values (which only have quarter resolution in YUV4:2:0)
                            Y = (ary[:ylen]).reshape((fheight, fwidth))
                            U = (ary[ylen:-uvlen]).reshape((fheight // 2, fwidth // 2)).repeat(2, axis=0).repeat(2, axis=1)
                            V = (ary[-uvlen:]).reshape((fheight // 2, fwidth // 2)).repeat(2, axis=0).repeat(2, axis=1)
                            # Stack the channels together and crop to the actual resolution
                            yuv = numpy.dstack((Y, U, V))[:height,:width,:]
                    self._yuv = yuv
        return self._yuv

    @property
    def rgb(self):
        """retrieve the image a RGB array. Takes 13s"""
        if  self._rgb is None:
            if colors is None:
                YUV = self.yuv.astype(numpy.int16)
            with self.sem:
                if self._rgb is None:
                    if colors:
                        resolution = self.camera_meta.get("resolution", (640, 480))
                        self._rgb, self._histograms = colors.yuv420_to_rgb16(self.data, resolution)
                    else:
                        YUV[:,:, 0] = YUV[:,:, 0] - 16  # Offset Y by 16
                        YUV[:,:, 1:] = YUV[:,:, 1:] - 128  # Offset UV by 128
                        # Calculate the dot product with the matrix to produce RGB output,
                        # clamp the results to byte range and convert to bytes
                        self._rgb = (YUV.dot(self.YUV2RGB) * 257.0).clip(0, 65535).astype(numpy.uint16)
        return self._rgb

    @property
    def histograms(self):
        """Calculate the 4 histograms with Y,R,G,B"""
        if  self._histograms is None:
            if colors is None:
                histograms = numpy.zeros((4, 256), numpy.int32)
                histograms[0] = numpy.bincount(self.yuv[:,:, 0].ravel(), minlength=256)
                histograms[1] = numpy.bincount(self.rgb[:,:, 0].ravel(), minlength=256)
                histograms[2] = numpy.bincount(self.rgb[:,:, 1].ravel(), minlength=256)
                histograms[3] = numpy.bincount(self.rgb[:,:, 2].ravel(), minlength=256)
                self._histograms = histograms
            else:
                rgb = self.rgb
        return self._histograms

    @classmethod
    def load(cls, fname):
        """load the raw data on one side and the header on the other"""

        with open(fname) as f:
            new = cls(f.read())
        jf = fname[:-3] + "json"
        if os.path.exists(jf):
            with open(jf) as f:
                new.camera_meta = json.load(f)
        if "index" in new.camera_meta:
            new.index = new.camera_meta["index"]
        return new

    def save(self):
        "Save the data as YUV raw data"
        fname = self.get_date_time() + ".yuv"
        with open(fname, "w") as f:
            f.write(self.data)
        fname = self.get_date_time() + ".json"
        comments = OrderedDict((("index", self.index),))
        if self.position:
            comments["pan"] = self.position.pan
            comments["tilt"] = self.position.tilt
        if self.gravity:
            comments["gx"] = self.gravity.x
            comments["gy"] = self.gravity.y
            comments["gz"] = self.gravity.z
        comments.update(self.camera_meta)
        with open(fname, "w") as f:
            f.write(json.dumps(comments, indent=4))
        logger.info("Saved YUV raw data %i %s", self.index, fname)


class StreamingOutput(object):
    """This class handles the stream, it re-cycles a BytesIO and provides frames"""

    def __init__(self, size):
        """Constructor

        :param size: size of an image in bytes. 
        For YUV, it is 1.5x the number of pixel of the padded image.
        """
        self.size = size
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = threading.Condition()

    def write(self, buf):
        res = self.buffer.write(buf)
        if self.buffer.tell() >= self.size:
            # image complete
            self.buffer.truncate(self.size)
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            with self.condition:
                self.frame = Frame(self.buffer.getvalue())
                self.condition.notify_all()
            self.buffer.seek(0)
        else:
            print("Incomplete buffer of %i bytes" % self.buffer.tell())
        return res


class CameraSimple(threading.Thread):
    "A simple camera class which continuously saves images"

    def __init__(self, resolution=(4056, 3040), framerate=1, sensor_mode=3,
                 # avg_ev=21, avg_wb=31, histo_ev=None,
                 wb_red=None, wb_blue=None,
                 quit_event=None, queue=None, config_queue=None,
                 folder="/tmp", index_callable=None):
        """This thread handles the camera: simple camera saving data to 
        
        """
        self.wb_red = wb_red or []
        self.wb_blue = wb_blue or []

        threading.Thread.__init__(self, name="Camera")
        signal.signal(signal.SIGINT, self.quit)
        self.quit_event = quit_event or threading.Event()
        self.folder = folder
        self.queue = queue
        self.index_callable = index_callable
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

    def get_filename(self):
        if callable(self.index_callable):
            new_index = self.index_callable()
            if self.last_index == new_index:
                self.last_subindex += 1
            else:
                self.last_subindex = 0
                self.last_index = new_index
            return f"{self.last_index:05d}-{self.last_subindex}.jpg"
        else:
            return str(get_isotime() + ".jpg")

    def get_config(self):
        config = OrderedDict([("resolution", tuple(self.camera.resolution)),
                              ("framerate", float(self.camera.framerate)),
                              ("sensor_mode", self.camera.sensor_mode),
                              # ("avg_ev", self.avg_ev),
                              # ("avg_wb", self.avg_wb),
                              # ("hist_ev", self.histo_ev),
                              # ("wb_red", self.wb_red),
                              # ("wb_blue", self.wb_blue)

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
        logger.info("warming up the camera for %ss", delay)
        framerate = self.camera.framerate
        self.camera.awb_mode = "auto"
        self.camera.exposure_mode = "auto"
        self.camera.framerate = 10
        while time.time() < end:
            rg, bg = self.camera.awb_gains
            rg = float(rg)
            bg = float(bg)
            if rg == 0.0:
                rg = 1.0
            if bg == 0.0:
                bg = 1.0
            self.wb_red.append(rg)
            self.wb_blue.append(bg)
            time.sleep(1)
        self.camera.framerate = framerate

    def run(self):
        "main thread activity"
        self.camera.awb_mode = "auto"
        self.camera.exposure_mode = "auto"
        while not self.quit_event.is_set():
            filename = self.get_filename()
            fullname = os.path.join(self.folder, filename)
            print(fullname)
            with self.lock:
                before = time.time()
                self.camera.capture(fullname)
                after = time.time()
            metadata = self.get_metadata()
            metadata["filename"] = filename
            metadata["time_before"] = before
            metadata["time_after"] = after
            self.queue.put(metadata)
            time.sleep(1)
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
        else:
            metadata['iso_calc'] = 100.0 * metadata["analog_gain"] * metadata["digital_gain"]
        return metadata


class Camera(threading.Thread):
    "A class for acquiring continusly images..."

    def __init__(self, resolution=(3280, 2464), framerate=1, sensor_mode=3,
                 avg_ev=21, avg_wb=31, histo_ev=None, wb_red=None, wb_blue=None,
                 quit_event=None, queue=None, config_queue=None):
        """This thread handles the camera
        
        """
        threading.Thread.__init__(self, name="Camera")
        signal.signal(signal.SIGINT, self.quit)
        self.quit_event = quit_event or threading.Event()
        self._can_record = threading.Event()
        self._done_recording = threading.Event()
        self._done_recording.set()
        self._can_record.set()
        self.queue = queue or Queue()
        self.config_queue = config_queue or Queue()
        self.avg_ev = avg_ev
        self.avg_wb = avg_wb
        self.histo_ev = histo_ev or []
        self.wb_red = wb_red or []
        self.wb_blue = wb_blue or []
        raw_size = (((resolution[0] + 31) & ~(31)) * ((resolution[1] + 15) & ~(15)) * 3 // 2)
        self.stream = StreamingOutput(raw_size)
        self.camera = PiCamera(resolution=resolution, framerate=framerate, sensor_mode=sensor_mode)

    def __del__(self):
        self.camera = self.stream = None

    def quit(self, *arg, **kwarg):
        "quit the main loop and end the thread"
        self.quit_event.set()

    def pause(self, wait=True):
        "pause the recording, wait for the current value to be acquired"
        self._can_record.clear()
        if wait:
            self._done_recording.wait()

    def resume(self):
        "resume the recording"
        self._can_record.set()

    def get_config(self):
        config = OrderedDict([("resolution", tuple(self.camera.resolution)),
                              ("framerate", float(self.camera.framerate)),
                              ("sensor_mode", self.camera.sensor_mode),
                              ("avg_ev", self.avg_ev),
                              ("avg_wb", self.avg_wb),
                              ("hist_ev", self.histo_ev),
                              ("wb_red", self.wb_red),
                              ("wb_blue", self.wb_blue)])
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

    def set_analysis(self, do_analysis):
        if do_analysis:
            self.camera.awb_mode = "off"  # "auto"
            self.camera.exposure_mode = "off"  # night" #"auto"
        else:
            self.camera.awb_mode = "auto"
            self.camera.exposure_mode = "auto"

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
        else:
            metadata['iso_calc'] = 100.0 * metadata["analog_gain"] * metadata["digital_gain"]
        return metadata

    def warm_up(self, delay=10):
        "warm up the camera"
        logger.info("warming up the camera for %ss", delay)
        framerate = self.camera.framerate
        self.camera.awb_mode = "auto"
        self.camera.exposure_mode = "auto"
        self.camera.framerate = 10
        for i in range(delay):
            rg, bg = self.camera.awb_gains
            rg = float(rg)
            bg = float(bg)
            if rg == 0.0:
                rg = 1.0
            if bg == 0.0:
                bg = 1.0
            self.wb_red.append(rg)
            self.wb_blue.append(bg)
            time.sleep(1)
        self.camera.framerate = framerate

    def run(self):
        "main thread activity"
        # self.camera.awb_mode = "off" # "auto"
        # self.camera.exposure_mode = "off" #night" #"auto"
        self._done_recording.clear()
        for foo in self.camera.capture_continuous(self.stream, format='yuv'):
            self._done_recording.set()
            if self.stream.frame is not None:
                frame = self.stream.frame
                logger.debug("Acquired %s", frame)
                frame.camera_meta = self.get_metadata()
                self.queue.put(frame)
            else:
                logger.info("No frame acquired")
            if self.quit_event.is_set():
                break
            # update the camera settings if needed:
            # Disabled for now at trajlaps level
            if not self.config_queue.empty():
                while not self.config_queue.empty():
                    evrb = self.config_queue.get()
                    if evrb.red:
                        self.wb_red.append(evrb.red)
                        self.wb_blue.append(evrb.blue)
                    if evrb.ev:
                        self.histo_ev.append(evrb.ev)
                self.config_queue.task_done()
                self.update_expo()
            self._can_record.wait()
            self._done_recording.clear()
        self.camera.close()

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
        # if self.camera.revision == "imx219":
        #    speed *= 1.84
        framerate = float(self.camera.framerate)
        logger.info("Update speed: %s %s", speed, framerate)

        if speed > framerate:
            self.camera.shutter_speed = int(1000000. / framerate / speed)
            self.camera.iso = 100
        elif speed > framerate * 2:
            self.camera.shutter_speed = int(2000000. / framerate / speed)
            self.camera.iso = 200
        elif speed > framerate * 4:
            self.camera.shutter_speed = int(4000000. / framerate / speed)
            self.camera.iso = 400
        else:
            self.camera.shutter_speed = min(int(8000000. / framerate / speed), int(1000000 / framerate))
            self.camera.iso = 800
#        #TODO: how to change framerate ? maybe start with low


class Saver(threading.Thread):
    "This thread is in charge of saving the frames arriving from the queue on the disk"

    def __init__(self, folder="/mnt", queue=None, quit_event=None):
        threading.Thread.__init__(self, name="Saver")
        self.queue = queue or Queue()
        self.quit_event = quit_event or threading.Signal()
        self.folder = os.path.abspath(folder)
        if not os.path.exists(self.folder):
            logger.warning("Creating folder %s", self.folder)
            os.makedirs(self.folder)

    def run(self):
        while not self.quit_event.is_set():
            t0 = time.time()
            frames = self.queue.get()
            if frames:
                frame = frames.pop()
                if not frame:
                    continue
                comments = OrderedDict((("index", frame.index),
                                        ("summed", 1)))
                exposure_speed = frame.camera_meta.get("exposure_speed", 1)
                RGB16 = frame.rgb
                if exposure_speed > 62000.0:  # 1/16 seconde
                # 2e5/frame.camera_meta.get("framerate"):
                    while frames:
                        other = frames.pop()
                        # merge in linear RGB space
                        summed, over = sRGB.sum(RGB16, other.rgb)
                        if over:
                            break
                        else:
                            RGB16 = summed
                        comments["summed"] += 1
                        exposure_speed += other.camera_meta.get("exposure_speed", 1)
                frames = None
                gc.collect()
                name = os.path.join(self.folder, frame.get_date_time() + ".jpg")
                logger.info("Save frame #%i as %s sum of %i", frame.index, name, comments["summed"])
                rgb8 = sRGB.compress(RGB16)
                Image.fromarray(rgb8).save(name, quality=90, optimize=True, progressive=True)
                exif = pyexiv2.ImageMetadata(name)
                exif.read()
                speed = Fraction(int(exposure_speed), 1000000)
                iso = int(frame.camera_meta.get("iso_calc"))
                exif["Exif.Photo.FNumber"] = Fraction(int(frame.camera_meta.get("aperture") * 100), 100)
                exif["Exif.Photo.ExposureTime"] = speed
                exif["Exif.Photo.ISOSpeedRatings"] = iso
                if frame.position:
                    comments["pan"] = frame.position.pan
                    comments["tilt"] = frame.position.tilt
                if frame.gravity:
                    comments["gx"] = frame.gravity.x
                    comments["gy"] = frame.gravity.y
                    comments["gz"] = frame.gravity.z
                comments.update(frame.camera_meta)
                if frame.servo_status:
                    comments.update(frame.servo_status)
                exif.comment = json.dumps(comments)
                exif.write(preserve_timestamps=True)
            self.queue.task_done()
            logger.info("Saving of frame #%i took %.3fs, sum of %s", frame.index, time.time() - t0, comments["summed"])


class Analyzer(threading.Thread):
    "This thread is in charge of analyzing the image and suggesting new exposure value and white balance"

    def __init__(self, frame_queue=None, config_queue=None, quit_event=None):
        threading.Thread.__init__(self, name="Analyzer")
        self.queue = frame_queue or Queue()
        self.output_queue = config_queue or Queue()
        self.quit_event = quit_event or threading.Signal()
        # self.history = []
        # self.max_size = 100

        # i = numpy.arange(40)
        # j = 0.5 ** (0.25 * i)
        # k = ((j + 0.099) / 1.099) ** (1 / 0.45) * (235 - 16) + 16
        # m2 = j < 0.018
        # k[m2] = (235-16) / 4.5 * j[m2] + 16
        # kr = numpy.round(k).astype(int)
        # self.ukr = numpy.concatenate(([0], numpy.sort(numpy.unique(kr)), [256]))
        # start = -0.25*(self.ukr.size-1)+0.5
        # self.delta_expo = numpy.arange(start, 0.5, 0.25)
        # self.g19_2 = gaussian(19, 2)
        # self.g19_2 /= self.g19_2.sum()

    def run(self):
        """This executed in a thread"""
        target_rgb = 5e-4  # pixels at 99.5% should be white
        while not self.quit_event.is_set():
            frame = self.queue.get()
            t0 = time.time()
            ev = oldev = lens.calc_EV(1000000 / frame.camera_meta.get("exposure_speed", 1), iso=frame.camera_meta.get("iso_calc", 100))
            histo = frame.histograms
            if 1:  # for exposure calculation:
                ylin = histo[0]
                ymax = numpy.where(ylin)[0][-1]
                # logger.info("ymax: %s", ymax)
                if ymax > 1000:
                    logger.debug("exposition %s is correct to over %s", ev, ymax)
                    cs = ylin.cumsum()
                    lim = 16
                    lo_light = cs[lim - 1]
                    hi_light = cs[-1] - cs[-lim]
                    if hi_light > lo_light:  # over exposed
                        if lo_light == 0:
                            ev += 1
                        else:
                            log(1.0 * hi_light / lo_light, 2)
                        logger.info("image is over-exposed, let's shrink %s %s eV: %s->%s", lo_light, hi_light, oldev, ev)
                else:
                    ev += log(1.0 * ymax / ylin.size, 2)
                    logger.info("image is under exposed, let's boost it %s eV: %s->%s", ymax, oldev, ev)
            if 1:  # Calculation of the corrected white-balance
                csr = numpy.cumsum(histo[1])
                csg = numpy.cumsum(histo[2])
                csb = numpy.cumsum(histo[3])
                if (csr[-1] != csg[-1]) or (csg[-1] != csb[-1]):
                    logger.error("Different number of pixel in chanels R, G and B: %s", histo.sum(axis=-1))
                pos = csr[-1] * (1.0 - target_rgb)
                try:
                    pos_r = numpy.where(csr >= pos)[0][0]
                    pos_g = numpy.where(csg >= pos)[0][0]
                    pos_b = numpy.where(csb >= pos)[0][0]
                except IndexError as e:
                    logger.error("no awb %s, exposure to low ", e)
                    # self.queue.task_done()
                    # continue
                    pos_r = numpy.where(histo[1])[0][-1]
                    pos_g = numpy.where(histo[2])[0][-1]
                    pos_b = numpy.where(histo[3])[0][-1]
                rg, bg = frame.camera_meta.get("awb_gains", (1.0, 1.0))
                if rg == 0.0:
                    rg = 1.0
                if bg == 0.0:
                    bg = 1.0
                try:
                    red_gain = 1.0 * rg * pos_g / pos_r
                    blue_gain = 1.0 * bg * pos_g / pos_b
                    logger.info("Update Red: %s -> %s Blue %s -> %s r%s g%s b%s", rg, red_gain, bg, blue_gain, pos_r, pos_g, pos_b)
                    # awb = GainRedBlue(min(8, max(0.125, red_gain)), min(8, max(0.125, blue_gain)))
                except ZeroDivisionError:
                    logger.error("pos_r %s, pos_g %s, pos_b %s, rg %s, bg %s", pos_r, pos_g, pos_b, rg, bg)
                    red_gain = rg
                    blue_gain = bg
                    # awb = GainRedBlue(rg, bg)
            else:
                red_gain = None
                blue_gain = None
            now = time.time()
            awb = ExpoRedBlue(ev, min(8, max(0.125, red_gain)), min(8, max(0.125, blue_gain)))
            self.output_queue.put(awb)
            self.queue.task_done()
            logger.info("Analysis of frame #%i took: %.3fs, delay since acquisition: %.3fs", frame.index, now - t0, now - frame.timestamp)

