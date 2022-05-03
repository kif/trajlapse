#!/usr/bin/python
from __future__ import division
import Adafruit_PCA9685
from collections import OrderedDict
pwm = Adafruit_PCA9685.PCA9685()
# Set frequency to 50hz, good for servos.
# gives a period of 20ms/cycle
FREQ = 330
RESOLUTION = 4096
pwm.set_pwm_freq(FREQ)

class Servo(object):
    """Represent a servomotor"""
    PROTECT = False 
    #set to True if the range should be restricted to protect the servo

    # Specification of the servo deg -> us
    SPEC={  4: 581,#us
            17: 2380}

    def __init__(self, pin, offset=0, reverse=False):
        """
        :param pin: pin number on the PCA9685 card
        :param offset: 
        """
        self.pin = pin
        self.offset = offset
        self.reverse = reverse
        self.angle_req = None
        self.angle_set = None
        angles = list(self.SPEC.keys())
        amax = max(angles)
        amin = min(angles)
        tick = 1.0e6 / FREQ / RESOLUTION
        tickmax = self.SPEC[amax] / tick
        tickmin = self.SPEC[amin] / tick
        self.slope = (tickmax - tickmin) / (amax - amin)
        self.inter = tickmin - amin * self.slope 
        

    def move(self, angle):
        "Set the servo motor to the given angle in degree"
        ticks = int(round(self.calc(angle)))
        pwm.set_pwm(self.pin, 0, ticks)
        self.angle_req = angle
        angle_set = (ticks - self.inter) / self.slope + self.offset
        self.angle_set = -angle_set if self.reverse else angle_set
    
    def get_config(self):
        res = OrderedDict([("name", self.__class__.__name__),
                           ("requested", self.angle_req),
                           ("set", self.angle_set)])
        return res
        
    def off(self):
        "switch off the servo"
        pwm.set_pwm(self.pin, 0, 0)
        self.angle_req = None
        self.angle_set = None

    def calc(self, angle):
        "calculate the number of ticks for a requested angle"
        if self.reverse:
            angle=-angle
        if self.offset:
            angle -= self.offset
        if self.PROTECT:
            #Protect for out of bounds ...
            angles = list(self.SPEC.keys())
            angle = max(angle, min(angles))
            angle = min(angle, max(angles))
        return (self.inter + self.slope*angle)


SG90 = Servo


class M15S(Servo):
    # deg -> us
    SPEC = { -90: 482,
            180: 2604,}


class MG90s(Servo):
    # deg -> us
    SPEC={  4: 581,#us
          176: 2380}

class MG92b(Servo):
    SPEC = { 14.300477656208258: 752.7669270833334,# deg -> us
             165.69952234379173: 2469.889322916667}

class TSS11MGB(Servo):
    SPEC={  2.17: 691.732,#deg->us
            175.23: 2685.55}
    PROTECT = True

pan = M15S(14, reverse=True, offset=-90)
#tilt = SG90(15, reverse=False, offset=-90)
tilt = MG92b(15, reverse=False, offset=-80+26)
#tss = TSS11MGB(12)
