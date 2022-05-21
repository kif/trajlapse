#!/usr/bin/python3
# Accelerometer API in top of for MMA8451 accelerometer.
#
# See 'LICENSE'  for copying
# Author: Jerome Kieffer
#
# Keep in mind to echo 1> /sys/module/i2c_bcm2708/parameters/combined

import time
import threading
import collections
Acceleration = collections.namedtuple("Acceleration", "x y z t")
import logging

# logging.basicConfig()
logger = logging.getLogger('accelero')
try:
    import mma8451
except:
    logger.error("Unable to import mma8451 module")


class Accelerometer(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self._gx = self._gy = self._gz = self.timestamp = None
        self.sem = threading.Semaphore()  # used to ensure consistent state intenally
        self.lock = threading.Semaphore()  # used to lock recording from outside
        self.quit = threading.Event()
        self.mma8451 = mma8451.MMA8451(sensor_range=mma8451.RANGE_2G, data_rate=mma8451.BW_RATE_1_56HZ, debug=False)

    def get(self):
        with self.sem:
            res = Acceleration(self._gx, self._gy, self._gz, self.timestamp)
        return res

    def run(self):
        self.mma8451.set_resolution()
        while not self.quit.is_set():
            with self.lock:
                t0 = round(time.time(), 3)  # ms precision
                axes = self.mma8451.get_axes_measurement()
                with self.sem:
                    self._gx = axes["x"]
                    self._gy = axes["y"]
                    self._gz = axes["z"]
                    self.timestamp = t0
            time.sleep(0.1)

    @property
    def gx(self):
        with self.sem:
            return self._gx

    @property
    def gy(self):
        with self.sem:
            return self._gy

    @property
    def gz(self):
        with self.sem:
            return self._gz


if __name__ == "__main__":
    logging.basicConfig()
    acc = Accelerometer()
    acc.start()
    time.sleep(1)
    g = acc.get()
    print(g, time.time() - g.t)
    acc.quit.set()
