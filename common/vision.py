import wpilib
import math

from components import swervedrive
from networktables import NetworkTables

class Vision():

    table = NetworkTables.getTable('limelight')

    KpHorizontal = -0.1 # Proportional control constant for adjustment in horizontal
    KpVertical = -0.1 # Proportional control constant for adjustment in vertical

    cam_height = 30 # Camera's height from the ground in cm
    cam_angle = 45 # Camera's angle from the horizontal in degrees

    target_height = 249 # Target's mid-point's height from the ground in cm
    target_distance = 30 # Desired distance from the wall to shoot
    
    tv = 0 # Whether the limelight has any valid targets
    tx = 0 # Horizontal offset from croshair to target in degrees
    ty = 0 # Vertical offset from croshair to target in degress

    def getValues(self):
        self.tv = self.table.getNumber('tv', 0)
        self.tx = self.table.getNumber('tx', 0)
        self.ty = self.table.getNumber('ty', 0)

    @staticmethod
    def degree_to_rad(degree):
        return degree * math.pi / 180

    def getDistance(self):
        '''
        Calculate the distance between the camera and the target wall.
        :return: Distance in cm
        '''
        # https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
        
        self.getValues()

        a1 = self.degree_to_rad(self.ty)
        a2 = self.degree_to_rad(self.cam_angle)

        distance = (self.target_height - self.cam_height) / math.tan(a1 + a2)
        return distance

    def verticalAdjust(self):
        '''
        Get the required vertical adjustment (drive) to align.
        :return: [-1, 1]
        '''
        current_distance = self.getDistance()
        error = self.KpVertical * (self.target_distance - current_distance)
        adjust = max(min(error, 1), -1)
        return adjust

    def horizontalAdjust(self):
        '''
        Get the required horizontal adjustment (rotate) to align.
        :return: [-1, 1]
        '''
        self.getValues()

        error = self.KpHorizontal * self.tx
        adjust = max(min(error, 1), 0)
        return adjust

    def updateTable(self):
        self.table.putNumber('Rotate', self.horizontalAdjust())
        self.table.putNumber('Drive', self.verticalAdjust())

