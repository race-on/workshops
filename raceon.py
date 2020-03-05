import time
import os

import RPi.GPIO as GPIO


class Encoders:
    
    def __init__(self, left_channel, right_channel):
        self.LEFT  = left_channel
        self.RIGHT = right_channel
        
        self.counter_left  = 0
        self.counter_right = 0
        
        # Setup the pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LEFT,  GPIO.IN)
        GPIO.setup(self.RIGHT, GPIO.IN)
        
        GPIO.add_event_detect(self.LEFT,  GPIO.BOTH, callback=self)
        GPIO.add_event_detect(self.RIGHT, GPIO.BOTH, callback=self)
        

    def __call__(self, channel):
        
        if channel == self.LEFT:
            self.counter_left  += 1
            
        if channel == self.RIGHT:
            self.counter_right += 1
        
        
    def read(self):
        l, r = self.counter_left, self.counter_right

        self.counter_left  = 0
        self.counter_right = 0

        return l, r

class PWM(object):
    """
    A class to work with the Linux PWM driver sysfs interface
    """

    def __init__(self, channel=0, chip=0):
        """ Specify channel and chip when creating an instance
        The Linux kernel driver exports a sysfs interface like this
            /sys/class/pwm/pwmchip<chip>/pwm<channel>
        A <chip> can have multiple <channels>.
        The channel and chip are determined by the kernel driver.
        For example the two PWM timers from the RPi kernel driver
        show up like this
            /sys/class/pwm/pwmchip0/pwm0
            /sys/class/pwm/pwmchip0/pwm1
        To use the RPi timers create instances this way
            pwm0 = PWM(0) or PWM(0,0)
            pwm1 = PWM(1) or PWM(1,0)
        """
        self._channel = channel
        self._chip = chip
        self.base = '/sys/class/pwm/pwmchip{:d}'.format(self._chip)
        self.path = self.base + '/pwm{:d}'.format(self._channel)

        if not os.path.isdir(self.base):
            raise FileNotFoundError('Directory not found: ' + self.base)

    # enable class as a context manager
    def __enter__(self):
        self.export()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.enable = False
        self.inversed = False
        self.unexport()
        return

    def export(self):
        """Export the channel for use through the sysfs interface.
        Required before first use.
        """
        if not os.path.isdir(self.path):
            with open(self.base + '/export', 'w') as f:
                f.write('{:d}'.format(self._channel))

    def unexport(self):
        """Unexport the channel.
        The sysfs interface is no longer usable until it is exported again.
        """
        if os.path.isdir(self.path):
            with open(self.base + '/unexport', 'w') as f:
                f.write('{:d}'.format(self._channel))

    @property
    def channel(self):
        """The channel used by this instance.
        Read-only, set in the constructor.
        """
        return self._channel

    @property
    def chip(self):
        """The chip used by this instance.
        Read-only, set in the constructor.
        """
        return self._chip

    @property
    def period(self):
        """The period of the pwm timer in nanoseconds."""
        with open(self.path + '/period', 'r') as f:
            value = f.readline().strip()

        return int(value)

    @period.setter
    def period(self, value):
        with open(self.path + '/period', 'w') as f:
            f.write('{:d}'.format(value))

    @property
    def duty_cycle(self):
        """The duty_cycle (the ON pulse) of the timer in nanoseconds."""
        with open(self.path + '/duty_cycle', 'r') as f:
            value = f.readline().strip()

        return int(value)

    @duty_cycle.setter
    def duty_cycle(self, value):
        with open(self.path + '/duty_cycle', 'w') as f:
            f.write('{:d}'.format(value))

    @property
    def enable(self):
        """Enable or disable the timer, boolean"""
        with open(self.path + '/enable', 'r') as f:
            value = f.readline().strip()

        return True if value == '1' else False

    @enable.setter
    def enable(self, value):
        with open(self.path + '/enable', 'w') as f:
            if value:
                f.write('1')
            else:
                f.write('0')

    @property
    def inversed(self):
        """normal polarity or inversed, boolean"""
        with open(self.path + '/polarity', 'r') as f:
            value = f.readline().strip()

        return True if value == 'inversed' else False

    @inversed.setter
    def inversed(self, value):
        with open(self.path + '/polarity', 'w') as f:
            if value:
                f.write('inversed')
            else:
                f.write('normal')
                

class Car:
    '''
    A class to control an ackerman drive car using a motor and a servo
    '''

    def __init__(self, motor_pin=None, servo_pin=None, servo_left=1000, servo_mid=1500, servo_right=2000, motor_reverse=False):
        assert motor_pin is not None, "motor_pin is not defined"
        assert servo_pin is not None, "servo_pin is not defined"

        if servo_left < servo_right:
            assert servo_left < servo_mid < servo_right, "servo_mid is not in between servo_left and servo_right"
        else:
            assert servo_left > servo_mid > servo_right, "servo_mid is not in between servo_left and servo_right"

        # setup motor
        self.motor = PWM(motor_pin)
        self.motor.period = 20000000

        if motor_reverse:
            self.MOTOR_REVERSE = True
            self.MOTOR_BRAKE = 1500
            self.MOTOR_MIN = 1000
            self.MOTOR_MAX = 2000
        
        else:
            self.MOTOR_REVERSE = False
            self.MOTOR_MIN = self.MOTOR_BRAKE = 1000
            self.MOTOR_MAX = 2000

        # setup servo
        self.servo = PWM(servo_pin)
        self.servo.period = 20000000
        
        self.SERVO_MID = servo_mid
        self.SERVO_MIN = servo_left
        self.SERVO_MAX = servo_right

    def _map(self, value, from_min, from_max, to_min, to_max):
        from_range = from_max - from_min
        to_range = to_max - to_min

        scaled_value = float(value - from_min) / float(from_range)

        return int(to_min + (scaled_value * to_range))
    
    def _limit(self, value, min_, max_):
        if value < min_:
            return min_
        elif value > max_:
            return max_
        else:
            return value


    def enable(self):
        self.motor.duty_cycle = self.MOTOR_BRAKE * 1000
        self.servo.duty_cycle = self.SERVO_MID * 1000

        self.motor.enable = True
        self.servo.enable = True
    
    def disable(self):
        self.motor.duty_cycle = self.MOTOR_BRAKE * 1000
        self.servo.duty_cycle = self.SERVO_MID * 1000

        self.motor.enable = False
        self.servo.enable = False
    
    def brake(self):
        self.motor.duty_cycle = self.MOTOR_BRAKE * 1000

    def speed(self, _speed):
        if self.MOTOR_REVERSE:
            _speed = self._limit(_speed, -1000, 1000)
            self.motor.duty_cycle = self._map(_speed, -1000, 1000, self.MOTOR_MIN, self.MOTOR_MAX) * 1000
        
        else:
            _speed = self._limit(_speed, 0, 1000)
            self.motor.duty_cycle = self._map(_speed, 0, 1000, self.MOTOR_MIN, self.MOTOR_MAX) * 1000

    def steer(self, _steer):
        _steer = self._limit(_steer, -1000, 1000)
        if _steer < 0:
            self.servo.duty_cycle = self._map(_steer, -1000, 0, self.SERVO_MIN, self.SERVO_MID) * 1000
        else:
            self.servo.duty_cycle = self._map(_steer, 0, 1000, self.SERVO_MID, self.SERVO_MAX) * 1000