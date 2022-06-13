import math
import board
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import geopy.distance as geodist

import FlyingObject

class ServoPointer:
    '''
    The system of 2 servos that are used to point at the flying object
    '''
    def __init__(self, alt_stepper_obj, azi_stepper_obj):
        self.alt_stepper = alt_stepper_obj
        self.azi_stepper = azi_stepper_obj

        self._alt_stp_deg = 1 # Number of steps to move 1 degree
        self._azi_stp_deg = 1 # Number of steps to move 1 degree
        self._target_pos = [0, 0] # [target altitude, target azimuth]
        self._pos = [0,0]

    def set_gain(self, alt_steps_per_deg, azi_steps_per_deg):
        '''
        Set the number of steps per degree of motion for latitude and longitude
        '''
        self._alt_stp_deg = alt_steps_per_deg
        self._azi_stp_deg = azi_steps_per_deg

    def set_target(self, target_alt, target_azi):
        '''
        Set the target altitude and azimuth
        '''
        self._target_pos = [target_alt, target_azi]

    def update(self):
        '''
        Function to be run continuously such as to move the stepper
        in the correct direction by one step
        '''
        # Move the stepper in the correct direction
        self._point_stepper(0, self._alt_stp_deg, self.alt_stepper)
        self._point_stepper(1, self._azi_stp_deg, self.azi_stepper)

    def release(self):
        '''
        Cut power to motors
        '''
        self.alt_stepper.release()
        self.azi_stepper.release()

    def _point_stepper(self, idx, stps_per_deg, stepper_obj):
        '''
        Point the stepper one step/microstep in the right direction
        '''
        num_steps = (self._target_pos[idx] - self._pos[idx])*stps_per_deg

        if abs(num_steps) >= 1:
            if num_steps > 0:
                stepper_obj.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
                self._pos[idx] = self._pos[idx] + 1/stps_per_deg
            else:
                stepper_obj.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
                self._pos[idx] = self._pos[idx] - 1/stps_per_deg

        elif abs(num_steps) < 1:
            if num_steps > 0:
                stepper_obj.onestep(direction=stepper.FORWARD, style=stepper.MICROSTEP)
                self._pos[idx] = self._pos[idx] + 1/stps_per_deg
            else:
                stepper_obj.onestep(direction=stepper.BACKWARD, style=stepper.MICROSTEP)
                self._pos[idx] = self._pos[idx] - 1/stps_per_deg


class TurretGPS:
    '''
    USB GPS connected to Dom Turretto
    '''
    def __init__(self):
        self.lat = 0
        self.long = 0
        self.alt = 0

    def calc_direction(self, FO):
        '''
            Calculate the heading direction between Dom Turretto and the Flying object
            return tuple containing (altitude, azimuth) of target
        '''
        # Calculate the distance in north and east directions
        # Using the launch location of the rocket as the reference for latitude/longitude calculations
        north_start_distance = geodist.geodesic(
            (self.lat, FO.start_long),
            (FO.start_lat, FO.start_long) ).m
        east_start_distance = geodist.geodesic(
            (FO.start_lat, self.long),
            (FO.start_lat, FO.start_long) ).m

        # Add signs to these distances to convert to point in NE FOR relative to turret
        # Note that this will not work at the north pole, or at the international date line.
        if self.lat - FO.start_lat < 0:
            north_coord = north_start_distance + FO.pn
        else:
            north_coord = -north_start_distance + FO.pn

        if self.long - FO.start_long < 0:
            east_coord = east_start_distance + FO.pe
        else:
            east_coord = -east_start_distance + FO.pe

        # Assuming flat earth for down coord; if earth's spherical nature starts is a problem,
        # you should really start using something better than a random stepper driver on a Pi.
        down_coord = FO.start_alt - FO.pd - self.alt

        # Calculate altitude and azimuth from these coordinates, in degrees
        tgt_alt = math.atan(-down_coord/math.sqrt(north_coord**2 + east_coord**2))*180/math.pi
        tgt_azi = math.atan2(north_coord, east_coord)*180/math.pi

        return (tgt_alt, tgt_azi)



class Turret:
    '''
    Dom Turretto. Has a servo system, and a GPS receiver.
    '''
    def __init__(self, FO = 0):
        self._heading = 0 # Heading that the base is facing
        self._motor_kit = MotorKit(i2c=board.I2C())
        self.stepper_rig = ServoPointer(self._motor_kit.stepper1, self._motor_kit.stepper2)

        self.tgt_object = FO
        self.gps = TurretGPS()


    def set_heading(self, base_heading):
        '''
        Set the heading of the turret when it has been turned on
        '''
        self._heading = base_heading

    def set_tgt(self, tgt_alt = 0, tgt_azi = 0):
        '''
        Set the heading that the turret should point towards
        Set to the direction of the target object if set
        '''
        if self.tgt_object:
            tgt_dir = self.gps.calc_direction(self.tgt_object)
            self.stepper_rig.set_target(tgt_dir[0], tgt_dir[1]-self._heading)
        else:
            self.stepper_rig.set_target(tgt_alt, tgt_azi-self._heading)

    def update(self):
        '''
        Update the servos to continue pointing
        '''
        self.stepper_rig.update()
