import wpilib
import math

from components import swervedrive
from networktables import NetworkTables
from networktables.util import ntproperty

class Vision():
    # Main networktable
    table = NetworkTables.getTable('limelight')
    # Horizontal offset from croshair to target in degrees
    tx = ntproperty('/limelight/tx', 0)
    # Vertical offset from croshair to target in degress
    ty = ntproperty('/limelight/ty', 0)
    # Whether the limelight has any valid targets
    tv = ntproperty('/limelight/tv', 0)

    KpHorizontal = -0.6 # Proportional control constant for adjustment in horizontal
    KpVertical = -0.3 # Proportional control constant for adjustment in vertical

    cam_height = 38.5 # Camera's height from the ground in inches
    cam_angle = 70 # Camera's angle from the horizontal in degrees

    target_height = 98.25 # Target's mid-point's height from the ground in inches
    target_distance = 120 # Desired distance from the wall to shoot

    debug = True

    def getValues(self):
        '''
        Get values from the Limelight networktable.
        '''
        values = dict()
        values['tx'] = self.table.getNumber('tx', 0)
        values['ty'] = self.table.getNumber('ty', 0)
        values['tv'] = self.table.getNumber('tv', 0)

        return values

    @staticmethod
    def degree_to_rad(degree):
        return degree * math.pi / 180

    def getDistance(self):
        '''
        Calculate the distance between the camera and the target wall.
        :return: Distance in inch
        '''
        # https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
        
        a1 = self.degree_to_rad(self.ty)
        a2 = self.degree_to_rad(self.cam_angle)

        distance = (self.target_height - self.cam_height) / math.tan(a1 + a2)
        return distance

    def verticalAdjust(self):
        '''
        Return the required vertical adjustment (drive) to align.
        :return: Clamped error [-1, 1]
        '''
        error = self.ty
        if error < 0.5 and error > -0.5:
            error = 0
        adjust = max(min(error, 1), -1)
        return adjust

    def horizontalAdjust(self):
        '''
        Get the required horizontal adjustment (rotate) to align.
        :return: Clamped error [-1, 1]
        '''
        error = self.tx
        if error < 0.5 and error > -0.5:
            error = 0
        adjust = max(min(error, 1), -1)
        return adjust

    def updateTable(self):
        if self.debug:
            self.table.putNumber('Drive', self.verticalAdjust())
            self.table.putNumber('Rotate', self.horizontalAdjust())
            self.table.putNumber('Distance', self.getDistance())
