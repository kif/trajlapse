#!/usr/bin/python3

"""Module for calculating the absolute exposure EV

Specifications of the lense used:
LS-2717CS
* focal length: 4mm,
* aperture: f/1.4
* format 1"/2.5 (the RPi HD camera has a sensor of 1"/2.3
* weight: 41.2g
"""

from math import log


class Exposure:
    """Class for calculating absolute Exposure Value
    from speed & gain et vice-versa"""

    def __init__(self, focal=4.0, aperture=1.4):
        """
        :param focal: Focal length in mm
        :param aperture
        """
        self.focal = focal
        self.aperture = aperture

    def __repr__(self):
        return "Exposure calculation for lens with f'=%smm and aperture F/%s" % (self.focal, self.aperture)

    def calc_EV(self, speed, gain=None, iso=100):
        """absolute exposure calculator

        :param speed: exposure speed in 1/s
        :param gain: actual gain of the camera (digital*analog)
        :param iso: equivalent to 100*gain
        :return:  the absolute exposure value
        """
        if gain is None:
            gain = iso / 100.
        return log(1.0 * self.aperture ** 2 * speed / gain , 2.0)

    def calc_speed(self, ev):
        """Calculate the speed needed at given exposure value, in 1/s"""
        return pow(2.0, ev) / self.aperture ** 2


lens = Exposure()

if __name__ == "__main__":

    for ev in range(20):
        s = lens.calc_speed(ev)
        if s < 1:
            print("For EV=%s: speed=%.3fs" % (ev, 1. / s))
        else:
            print("For EV=%s: speed=1/%.3fs" % (ev, s))
    for i in range(10):
        speed = 2 ** i
        print("For speed=1/%ss: EV=%.3f" % (speed, lens.calc_EV(speed)))

