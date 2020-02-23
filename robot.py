#!/usr/bin/env python3

import wpilib

import ctre

from magicbot import MagicRobot
from magicbot.magic_tunable import tunable

from robotpy_ext.autonomous.selector import AutonomousModeSelector

from networktables import NetworkTables
from networktables.util import ntproperty

from rev.color import ColorSensorV3, ColorMatch

from components import swervedrive, swervemodule

from collections import namedtuple
ModuleConfig = swervemodule.ModuleConfig

class MyRobot(MagicRobot):
    drive: swervedrive.SwerveDrive

    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule

    frontLeftModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=0, inverted=False, allow_reverse=True)
    frontRightModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=0, inverted=False, allow_reverse=True)
    rearLeftModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0, inverted=False, allow_reverse=True)
    rearRightModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=0, inverted=False, allow_reverse=True)

    def createObjects(self):
        # SmartDashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # Gamepad
        self.gamempad = wpilib.Joystick(0)

        # Drive Motors
        self.frontLeftModule_driveMotor = ctre.WPI_VictorSPX(5)
        self.frontRightModule_driveMotor = ctre.WPI_VictorSPX(8)
        self.rearLeftModule_driveMotor = ctre.WPI_VictorSPX(4)
        self.rearRightModule_driveMotor = ctre.WPI_VictorSPX(9)

        # Rotate Motors
        self.frontLeftModule_rotateMotor = ctre.WPI_VictorSPX(3)
        self.frontRightModule_rotateMotor = ctre.WPI_VictorSPX(14)
        self.rearLeftModule_rotateMotor = ctre.WPI_VictorSPX(2)
        self.rearRightModule_rotateMotor = ctre.WPI_VictorSPX(15)

        # Encoders
        self.frontLeftModule_encoder = wpilib.AnalogInput(0)
        self.frontRightModule_encoder = wpilib.AnalogInput(1)
        self.rearLeftModule_encoder = wpilib.AnalogInput(2)
        self.rearRightModule_encoder = wpilib.AnalogInput(3)

        # Shooter
        self.leftShooterMotor = ctre.WPI_VictorSPX(22)
        self.rightShooterMotor = ctre.WPI_VictorSPX(21)
        self.beltMotor = ctre.WPI_VictorSPX(20)
        self.intakeMotor = ctre.WPI_VictorSPX(11)

        # Wheel of Fortune
        self.wofMotor = ctre.WPI_VictorSPX(23)

        # Climber
        self.climbingMotor = ctre.WPI_VictorSPX(10)
        self.hookMotor = ctre.WPI_VictorSPX(24)

        # Color Sensor
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.colorMatcher = ColorMatch()

        self.kBlue = wpilib.Color(0.143, 0.427, 0.429)
        self.kGreen = wpilib.Color(0.197, 0.561, 0.240)
        self.kRed = wpilib.Color(0.561, 0.232, 0.114)
        self.kYellow = wpilib.Color(0.361, 0.524, 0.113)

        self.colorMatcher.addColorMatch(self.kBlue)
        self.colorMatcher.addColorMatch(self.kGreen)
        self.colorMatcher.addColorMatch(self.kRed)
        self.colorMatcher.addColorMatch(self.kYellow)

    def disabledPeriodic(self):
        self.update_sd()

    def autonomousInit(self):
        self.drive.allow_reverse = False
        self.drive.wait_for_align = True
        self.drive.threshold_input_vectors = True

    def autonomous(self):
        super().autonomous()

    def teleopInit(self):
        self.drive.flush()
        self.drive.allow_reverse = True
        self.drive.wait_for_align = False
        self.drive.squared_input = True
        self.drive.threshold_input_vectors = True

    def move(self, x, y, rcw):
        if self.gamempad.getRawButton(7):
            rcw *= 0.75

        self.drive.move(x, y, rcw)

    def teleopPeriodic(self):
        # Drive
        self.move(self.gamempad.getRawAxis(5) * -1, self.gamempad.getRawAxis(2) * -1, self.gamempad.getRawAxis(0))

        # Lock
        if self.gamempad.getRawButton(10):
            self.drive.request_wheel_lock = True

        # Vectoral Button Drive
        if self.gamempad.getRawButton(3):
            self.drive.set_raw_strafe(0.25)
        elif self.gamempad.getRawButton(2):
            self.drive.set_raw_strafe(-0.25)
        if self.gamempad.getRawButton(4):
            self.drive.set_raw_fwd(0.35)
        elif self.gamempad.getRawButton(1):
            self.drive.set_raw_fwd(-0.35)

        # Intake & Belt
        if self.gamempad.getRawButton(6):
            self.climbingMotor.set(1)
        else:
            self.climbingMotor.set(0)

        # Shooter
        self.leftShooterMotor.set(self.gamempad.getRawAxis(3))
        self.rightShooterMotor.set(self.gamempad.getRawAxis(3))

        # Color Sensor
        self.color = self.colorSensor.getColor()
        self.ir = self.colorSensor.getIR()

        self.sd.putNumber('color_blue', self.color.blue)
        self.sd.putNumber('color_green', self.color.green)
        self.sd.putNumber('color_red', self.color.red)

        self.match = self.colorMatcher.matchClosestColor(self.color, 0.5)
        # self.sd.putNumber('color_match', self.match)
        if (self.match == self.kBlue):
            self.sd.putString('color_match', 'Blue')
        elif (self.match == self.kGreen):
            self.sd.putString('color_match', 'Green')
        elif (self.match == self.kRed):
            self.sd.putString('color_match', 'Red')
        elif (self.match == self.kYellow):
            self.sd.putString('color_match', 'Yellow')

    def update_sd(self):
        self.drive.update_smartdash()

if __name__ == "__main__":
    wpilib.run(MyRobot)
