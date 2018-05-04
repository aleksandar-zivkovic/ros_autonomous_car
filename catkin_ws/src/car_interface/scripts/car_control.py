#!/usr/bin/python
import sys
import time
from Adafruit_PWM_Servo_Driver import PWM

SERVO_REFRESH_FREQUENCY = 50 # refresh rate 50 updates/s i.e. 20ms cycle

# The PWM signal has 4096 intervals for each cycle
# A pulse of length 220 @50Hz is (1/50)*220/4096 = 1.074ms
# A servo is controlled with a pulse length between 1ms and 2ms, i.e. a 1ms variation
# With a 50Hz / 20ms period, usable servo range is divided into 4096/20 = 204 steps
# For a better resolution the cycle frequency can be increased.
# With a 200Hz / 5ms period, usable servo range is divided into 4096/5 = 819 steps
# Not all servos (or motor controllers) can accept 200Hz refresh rate, but increasing it allows better resolution

servoMin = 220  # Min pulse length out of 4096
servoMax = 520  # Max pulse length out of 4096

steerLeft = 208     # Max steering deflection left TODO: config parameter
steerCenter = 312   # Center steering position TODO: config parameter
steerRight = 417    # Max steering deflection right TODO: config parameter

currentSteeringAngle = steerCenter
STEERING_ANGLE_STEP = 5
gas_pedal = 0

# Servo channel assignment on RPi Servo Hat
STEERING_PWM_CHANNEL = 0
MOTOR_PWM_CHANNEL = 15

# Motor control configuration values
MOTOR_STOP_PWM_VALUE = 307
MOTOR_MIN_PWM_VALUE = 290
MOTOR_DEADBAND_START_VALUE = 299
MOTOR_DEADBAND_STOP_VALUE = 323
MOTOR_MAX_PWM_VALUE = 340

MOTOR_MAX_THROTTLE_PERCENTAGE = 50
MOTOR_MAX_REVERSE_PERCENTAGE = -50
MOTOR_STEP_SIZE_PERCENT = 5
MOTOR_BRAKING_FORCE = 5
MOTOR_BRAKING_DELAY = 0.3


class CarControl(object):
    def __init__(self):
        self._pwm = PWM(0x40)    # I2C address 
        self._pwm.setPWMFreq(SERVO_REFRESH_FREQUENCY)
        self.motor_stop_and_center()
        self.reset_steering()
        return

    def more_gas(self):
        global gas_pedal
        if gas_pedal <= MOTOR_MAX_THROTTLE_PERCENTAGE:
            gas_pedal += MOTOR_STEP_SIZE_PERCENT
        else:
            gas_pedal = MOTOR_MAX_THROTTLE_PERCENTAGE
        self.throttle = gas_pedal
        return

    def reverse(self):
        global gas_pedal
        if gas_pedal >= MOTOR_MAX_REVERSE_PERCENTAGE:
            gas_pedal -= MOTOR_STEP_SIZE_PERCENT
        else:
            gas_pedal = MOTOR_MAX_REVERSE_PERCENTAGE
        self.throttle = gas_pedal
        return


    def active_braking(self):
        global gas_pedal
        # ACTIVE BRAKE OPPOSITE DIRECTION
        if gas_pedal > 0:
            self.throttle = -MOTOR_BRAKING_FORCE
        elif gas_pedal < 0:
            self.throttle = MOTOR_BRAKING_FORCE
        time.sleep(MOTOR_BRAKING_DELAY)
        self.throttle = 0
        gas_pedal = 0
        return

    def sensor_braking(self):
        global gas_pedal
        # ACTIVE BRAKE OPPOSITE DIRECTION
        self.throttle = -MOTOR_BRAKING_FORCE
        time.sleep(MOTOR_BRAKING_DELAY)
        self.throttle = 0
        gas_pedal = 0
        return

    def _get_motor_pwm_value(self):
        if self.throttle == 0:
            return MOTOR_STOP_PWM_VALUE
        elif self.throttle < 0:
            coeff = -self.throttle / 100.0
            min_value = MOTOR_DEADBAND_START_VALUE
            max_value = MOTOR_MIN_PWM_VALUE
        else:
            coeff = self.throttle / 100.0
            min_value = MOTOR_DEADBAND_STOP_VALUE
            max_value = MOTOR_MAX_PWM_VALUE
        assert 0.0 <= coeff <= 1.0
        diff = max_value - min_value
        # Interpolate between the min and max values
        return int(round(min_value + coeff * diff, 0))

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        assert -100 <= value <= 100
        self._throttle = value
        pwm_value = self._get_motor_pwm_value()
        print('Setting motor PWM to', pwm_value)
        self._pwm.setPWM(MOTOR_PWM_CHANNEL, 0, pwm_value)
        return

    def motor_stop_and_center(self):
        global currentSteeringAngle
        self.throttle = 0
        currentSteeringAngle = steerCenter
        return

    def stop_and_reset(self):
        self.throttle = 0
        self.active_braking()
        self.motor_stop_and_center()
        self.reset_steering()
        return

    def reset_steering(self):
        global currentSteeringAngle
        self._pwm.setPWM(STEERING_PWM_CHANNEL, 0, steerCenter)
        currentSteeringAngle = steerCenter
        return

    def steer_right(self):
        global currentSteeringAngle
        if currentSteeringAngle <= (steerRight-STEERING_ANGLE_STEP):
            currentSteeringAngle += STEERING_ANGLE_STEP
        else:
            currentSteeringAngle = steerRight
        self._pwm.setPWM(STEERING_PWM_CHANNEL, 0, currentSteeringAngle)
        return

    def steer_left(self):
        global currentSteeringAngle
        if currentSteeringAngle >= (steerLeft+STEERING_ANGLE_STEP):
            currentSteeringAngle -= STEERING_ANGLE_STEP
        else:
            currentSteeringAngle = steerLeft
        self._pwm.setPWM(STEERING_PWM_CHANNEL, 0, currentSteeringAngle)
        return

def main():
    cc = CarControl()
    for throttle, seconds in zip(sys.argv[1::2], sys.argv[2::2]):
          cc.throttle = int(throttle)
          seconds = float(seconds)
          print('Sleeping for', seconds)
          time.sleep(seconds)
    cc.motor_stop_and_center()

if __name__ == '__main__':
    main()
