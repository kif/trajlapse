import time
from collections import namedtuple, OrderedDict
from threading import Semaphore

Position = namedtuple("Position", ("pan", "tilt"))
Position.__repr__ = lambda self: f"Position(pan: {self.pan:.3f}°, tilt: {self.tilt:.3f}°)"


class Positioner:
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
        self._position = Position(360, 180)  # impossible position ...ensures long enough delays to move the first position
        self.locks = locks or []

    def __repr__(self):
        return str(self.position)

    def get(self):
        with self.lock:
            return self._position

    position = property(get)

    def goto(self, where):
        """Move the head to a given position
        :param where: Position
        :return: timestamp before and after move
        """
        result = OrderedDict()
        if where != self._position:
            pan, tilt = where
            with self.lock:
                for lock in self.locks:
                    lock.acquire()
                before = time.time()
                self.servo_pan.move(pan)
                self.servo_tilt.move(tilt)
                dp = abs(self._position.pan - where.pan)
                dt = abs(self._position.tilt - where.tilt)
                delay = max(dp * self.delay_60_pan / 60., dt * self.delay_60_tilt / 60.) + 1.0
                self._position = where
                time.sleep(delay)
                after = time.time()
                self.stop_motors()
                for lock in self.locks:
                    lock.release()

            result["pos_start"] = before
            result["pos_stop"] = after
        else:
            result["pos_start"] = result["pos_stop"] = time.time()
        result["pan"] = self.servo_pan.get_config()
        result["tilt"] = self.servo_tilt.get_config()

        return result

    def stop_motors(self):
        "Switch off both motors"
        try:
            self.servo_pan.off()
            self.servo_tilt.off()
        except IOError as err:
            print(err)

