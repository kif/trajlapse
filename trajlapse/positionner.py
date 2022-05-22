import time
from collections import namedtuple
from threading import Semaphore

Position = namedtuple("Position", ("pan", "tilt"))


class Positionner:
    "A class designed to move motors"

    def __init__(self, pan, tilt, locks=None):
        """Constructor
        
        :param pan: servomotor controling the panoramic movement (left<0, right>0)
        :param tilt: servomotor controling the tilt movement (down<0, up>0)
        :param locks: list of locks to prevent acquisition during movement
        """
        self.servo_pan = pan
        self.servo_tilt = tilt
        self.delay_60_pan = 0.3  # Time to move the servo by 60°
        self.delay_60_tilt = 0.2  # Time to move the servo by 60°
        self.lock = Semaphore()
        self._position = Position(None, None)

    def __repr__(self):
        return str(self.position)

    def get(self):
        with self.lock:
            return self._position

    position = property(get)

    def goto(self, where):
        """Move the head to a given position
        :param where: Position
        """
        if where != self._position:
            pan, tilt = where
            with self.lock:
                for lock in self.locks:
                    lock.acquire()
                self.servo_pan.move(pan)
                self.servo_tilt.move(tilt)
                dp = abs(self._position.pan - where.pan)
                dt = abs(self._position.tilt - where.tilt)
                delay = max(dp * self.delay_60_pan / 60., dt * self.delay_60_tilt / 60.) + 0.1
                self._position = where
                time.sleep(delay)
                for lock in self.locks:
                    lock.release()

