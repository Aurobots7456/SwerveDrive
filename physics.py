import math
import wpilib
import ctre

from networktables.util import ntproperty

from pyfrc.physics import motor_cfgs, tankmodel, drivetrains
from pyfrc.physics.units import units

class PhysicsEngine(object):

    FrontLeft_degrees = ntproperty("/SmartDashboard/drive/FrontLeft_Module/degrees", 0)
    FrontRight_degrees = ntproperty("/SmartDashboard/drive/FrontRight_Module/degrees", 0)
    RearLeft_degrees = ntproperty("/SmartDashboard/drive/RearLeft_Module/degrees", 0)
    RearRight_degrees = ntproperty("/SmartDashboard/drive/RearRight_Module/degrees", 0)

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller

    def initialize(self, hal_data):
        self.FrontLeft_encoder = 0
        self.FrontRight_encoder = 0
        self.RearLeft_encoder = 0
        self.RearRight_encoder = 0

    def update_sim(self, hal_data, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.
        
        :param hal_data: Data about motors and other components
        :param now: CuRearRightent time in ms
        :param tm_diff: Difference between cuRearRightent time and time when last checked
        """
        try:
            self.FrontLeft_encoder += hal_data['CAN'][1]['value'] * tm_diff * 20
            self.FrontRight_encoder += hal_data['CAN'][3]['value'] * tm_diff * 20
            self.RearLeft_encoder += hal_data['CAN'][5]['value'] * tm_diff * 20
            self.RearRight_encoder += hal_data['CAN'][7]['value'] * tm_diff * 20

            self.FrontLeft_encoder %= 5
            self.FrontRight_encoder %= 5
            self.RearLeft_encoder %= 5
            self.RearRight_encoder %= 5

            hal_data['analog_in'][0]['avg_voltage'] = self.FrontLeft_encoder
            hal_data['analog_in'][1]['avg_voltage'] = self.FrontRight_encoder
            hal_data['analog_in'][2]['avg_voltage'] = self.RearLeft_encoder
            hal_data['analog_in'][3]['avg_voltage'] = self.RearRight_encoder
        
        except Exception:
            pass

        try:
            hal_data['CAN'][0]['enc_position'] -= hal_data['CAN'][0]['value'] / 1023 * tm_diff * 300000
            hal_data['CAN'][1]['enc_position'] += hal_data['CAN'][1]['value'] / 1023 * tm_diff * 300000
            hal_data['CAN'][2]['enc_position'] -= hal_data['CAN'][2]['value'] / 1023 * tm_diff * 300000
            hal_data['CAN'][3]['enc_position'] += hal_data['CAN'][3]['value'] / 1023 * tm_diff * 300000

            FrontLeftMotor = -hal_data['CAN'][0]['value'] / 1023
            FrontRightMotor = hal_data['CAN'][1]['value'] / 1023
            RearLeftMotor = -hal_data['CAN'][2]['value'] / 1023
            RearRightMotor = hal_data['CAN'][3]['value'] / 1023

            vx, vy, vw = drivetrains.four_motor_swerve_drivetrain(RearLeftMotor, RearRightMotor, FrontLeftMotor, FrontRightMotor, self.RearLeft_degrees, self.RearRight_degrees, self.FrontLeft_degrees, self.FrontRight_degrees, x_wheelbase=2, y_wheelbase=3, speed=5)
            # vx, vy, vw = four_motor_swerve_drivetrain1(FrontLeftMotor, FrontRightMotor, RearLeftMotor, RearRightMotor, self.FrontLeft_degrees, self.FrontRight_degrees, self.RearLeft_degrees, self.RearRight_degrees, x_wheelbase=3, y_wheelbase=3.6, speed=9)
            self.physics_controller.vector_drive(vx, vy, vw, tm_diff)

        except Exception:
            pass

def four_motor_swerve_drivetrain1(FrontLeftMotor, FrontRightMotor, RearLeftMotor, RearRightMotor, FrontLeft_angle, FrontRight_angle, RearLeft_angle, RearRight_angle, x_wheelbase=2, y_wheelbase=2, speed=5):
    """
    Four motors that can be rotated in any direction.
    If any motors are inverted, then you will need to multiply that motor's
    value by -1.
    
    :param FrontLeftMotor: Left front motor value (-1 to 1); 1 is forward
    :param FrontRightMotor: Right front motor value (-1 to 1); 1 is forward
    :param RearLeftMotor: Left rear motor value (-1 to 1); 1 is forward
    :param RearRightMotor: Right rear motor value (-1 to 1); 1 is forward

    :param FrontLeft_angle: Left front motor angle in degrees (0 to 360)
    :param FrontRight_angle: Right front motor angle in degrees (0 to 360)
    :param RearLeft_angle: Left rear motor angle in degrees (0 to 360)
    :param RearRight_angle: Right rear motor angle in degrees (0 to 360)

    :param x_wheelbase: The distance in feet between right and left wheels.
    :param y_wheelbase: The distance in feet between forward and rear wheels.
    :param speed: Speed of robot in feet per second (see above)

    :return: Speed of robot in x (ft/s), Speed of robot in y (ft/s), clockwise rotation of robot (radians/s)
    """
    # Calculate speed of each wheel
    FrontLeft = FrontLeftMotor * speed
    FrontRight = FrontRightMotor * speed
    RearLeft = RearLeftMotor * speed
    RearRight = RearRightMotor * speed

    # Calculate angle in radians
    FrontLeft_rad = FrontLeft_angle * (math.pi / 180)
    FrontRight_rad = FrontRight_angle * (math.pi / 180)
    RearLeft_rad = RearLeft_angle * (math.pi / 180)
    RearRight_rad = RearRight_angle * (math.pi / 180)

    # Calculate wheelbase radius
    wheelbase_radius = math.hypot(x_wheelbase / 2.0, y_wheelbase / 2.0)

    # Calculates the Vx and Vy components
    # Sin and Cos inverted because forward is 0 on swerve wheels
    Vx = (math.sin(RearLeft_rad) * RearLeft) + (math.sin(RearRight_rad) * RearRight) + (math.sin(FrontLeft_rad) * FrontLeft) + (math.sin(FrontRight_rad) * FrontRight)
    Vy = (math.cos(RearLeft_rad) * RearLeft) + (math.cos(RearRight_rad) * RearRight) + (math.cos(FrontLeft_rad) * FrontLeft) + (math.cos(FrontRight_rad) * FrontRight)

    # To make left negative
    Vx *= -1

    # Adjusts the angle corrsponding to a diameter that is perpendicular to the radius (add or subtract 45deg)
    FrontLeft_rad = (FrontLeft_rad - (math.pi / 4)) % (2 * math.pi)
    FrontRight_rad = (FrontRight_rad + (math.pi / 4)) % (2 * math.pi)
    RearLeft_rad = (RearLeft_rad + (math.pi / 4)) % (2 * math.pi)
    RearRight_rad = (RearRight_rad - (math.pi / 4)) % (2 * math.pi)

    # Finds the rotational velocity by finding the torque and adding them up
    Vw = wheelbase_radius * ((math.cos(RearLeft_rad) * RearLeft) + (math.cos(RearRight_rad) * -RearRight) + (math.cos(FrontLeft_rad) * FrontLeft) + (math.cos(FrontRight_rad) * -FrontRight))

    Vx *= 0.25
    Vy *= 0.25
    Vw *= 0.25

    return Vx, Vy, Vw
