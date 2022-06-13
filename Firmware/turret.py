import time
import board
from adafruit_motorkit import MotorKit, stepper
import FlyingObject

class servoPointer:
    '''
    The system of 2 servos that are used to point at the flying object
    '''
    def __init__(self, alt_stepper_obj, azi_stepper_obj):
        self.kit = MotorKit(i2c=board.I2C())
        self.alt_stepper = alt_stepper_obj
        self.azi_stepper = azi_stepper_obj

        self._alt_stp_deg = 1 # Number of steps to move 1 degree
        self._azi_stp_deg = 1 # Number of steps to move 1 degree
        self._target_pos = [0, 0] # [target altitude, target azimuth]
        self._pos = [0,0]

    def set_gain(self, alt_steps_per_deg, azi_steps_per_deg):
        self.alt_stp_deg = alt_steps_per_deg
        self.azi_stp_deg = azi_steps_per_deg

    def set_target(self, target_alt, target_azi):
        self._target_pos = [target_alt, target_azi]
    
    def update(self):
        # Move the stepper in the correct direction
        

    def release(self):
        self.alt_stepper.release()
        self.azi_stepper.release()

    def _point_stepper(self, current_pos, target_pos, stps_per_deg, stepper_obj):
        '''
        Returns new position
        '''
        num_steps = (target_pos - current_pos)*stps_per_deg

        if (abs(num_steps) >= 1):
            if (num_steps>0):
                stepper_obj.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
            else:
                stepper_obj.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)

        elif (abs(num_steps) < 1):
            if (num_steps>0):
                stepper_obj.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
            else:
                stepper_obj.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)


class turretGPS:
    '''
    USB GPS connected to Dom Turretto
    '''
    def __init__(self):
        self.coords = 0, 0


class Turret:
    '''
    Dom Turretto. Has a servo system, and a GPS receiver.
    '''
    def __init__(self, longitude, latitude, height):
        self._long = longitude
        self._lat = latitude
        self._hei = height
        self._heading = 0 # Heading that the base is facing
        self.gps = turretGPS()
        self.rig = servoPointer()

    def set_heading(self, base_heading):
        self._heading = base_heading