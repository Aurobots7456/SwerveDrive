#!/usr/bin/env python3

import wpilib

import ctre

from magicbot import MagicRobot
from magicbot.magic_tunable import tunable

from robotpy_ext.autonomous.selector import AutonomousModeSelector

from networktables import NetworkTables
from networktables.util import ntproperty

from rev.color import ColorSensorV3, ColorMatch

from components import swervedrive, swervemodule, shooter, wof
from common import color_sensor, vision

from collections import namedtuple
ModuleConfig = swervemodule.ModuleConfig

class MyRobot(MagicRobot):
    drive: swervedrive.SwerveDrive
    shooter: shooter.Shooter
    wof: wof.WheelOfFortune

    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule

    frontLeftModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=2.97, inverted=True, allow_reverse=True)
    frontRightModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=2.69, inverted=False, allow_reverse=True)
    rearLeftModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0.18, inverted=True, allow_reverse=True)
    rearRightModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=4.76, inverted=False, allow_reverse=True)

    shooter_leftShooterMotor: ctre.WPI_VictorSPX
    shooter_rightShooterMotor: ctre.WPI_VictorSPX
    shooter_intakeMotor: ctre.WPI_VictorSPX
    shooter_beltMotor: ctre.WPI_VictorSPX

    vision: vision.Vision
    colorSensor: color_sensor.ColorSensor

    def createObjects(self):
        # SmartDashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # Gamepad
        self.gamempad = wpilib.Joystick(0)
        self.gamempad2 = wpilib.Joystick(1)

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
        self.frontRightModule_encoder = wpilib.AnalogInput(3)
        self.rearLeftModule_encoder = wpilib.AnalogInput(1)
        self.rearRightModule_encoder = wpilib.AnalogInput(2)

        # Shooter
        self.shooter_leftShooterMotor = ctre.WPI_VictorSPX(6)
        self.shooter_rightShooterMotor = ctre.WPI_VictorSPX(7)
        self.shooter_beltMotor = ctre.WPI_VictorSPX(11)
        self.shooter_intakeMotor = ctre.WPI_VictorSPX(0)

        # Wheel of Fortune
        self.wof_motor = ctre.WPI_VictorSPX(13)

        # Climber
        self.climbingMotor = ctre.WPI_VictorSPX(10)
        self.hookMotor = ctre.WPI_VictorSPX(1)

        # Color Sensor
        self.colorSensor = color_sensor.ColorSensor()

        # Vision
        self.vision = vision.Vision()

        # Limit Switch
        self.switch = wpilib.DigitalInput(0)

        # PDP
        self.pdp = wpilib.PowerDistributionPanel(0)

    def disabledPeriodic(self):
        self.update_sd()

    def autonomousInit(self):
        self.drive.flush()
        self.drive.threshold_input_vectors = True

    def autonomous(self):
        super().autonomous()

    def teleopInit(self):
        self.drive.flush()
        self.drive.squared_inputs = True
        self.drive.threshold_input_vectors = True

    def move(self, x, y, rcw):
        if self.gamempad.getRawButton(3):
            rcw *= 0.7

        self.drive.move(x, y, rcw)

    def teleopPeriodic(self):
        # Drive
        self.move(self.gamempad.getRawAxis(5), self.gamempad.getRawAxis(4), self.gamempad.getRawAxis(0))

        # Lock
        if self.gamempad.getRawButton(1):
            self.drive.request_wheel_lock = True

        # Vectoral Button Drive
        if self.gamempad.getPOV() == 0:
            self.drive.set_raw_fwd(0.35)
        elif self.gamempad.getPOV() == 180:
            self.drive.set_raw_fwd(-0.35)
        elif self.gamempad.getPOV() == 90:
            self.drive.set_raw_strafe(-0.35)
        elif self.gamempad.getPOV() == 270:
            self.drive.set_raw_strafe(0.35)

        # Climber
        if self.gamempad2.getRawButton(1):
            self.climbingMotor.set(1)
        else:
            self.climbingMotor.set(0)

        # Hook
        if self.gamempad2.getRawAxis(5) < 0 and not self.switch.get():
            self.hookMotor.set(self.gamempad2.getRawAxis(5))
        elif self.gamempad2.getRawAxis(5) > 0:
            self.hookMotor.set(self.gamempad2.getRawAxis(5))
        else:
            self.hookMotor.set(0)

        # Shooter
        if self.gamempad.getRawAxis(3) > 0:
            self.shooter.shoot()
        elif self.gamempad.getRawButton(6):
            self.shooter.align()
        elif self.gamempad.getRawButton(5) or self.gamempad2.getRawAxis(2) > 0:
            self.shooter.unload()
        elif self.gamempad.getRawAxis(2) > 0 or self.gamempad2.getRawAxis(3) > 0:
            self.shooter.intake()
        else:
            self.shooter.stop()

        # WoF
        if self.gamempad2.getRawButton(3):
            self.wof.handleFirstStage()
        elif self.gamempad2.getRawButton(2):
            self.wof.handleSecondStage()
        elif self.gamempad2.getRawButton(4):
            self.wof.reset()
        elif self.gamempad2.getRawButton(5):
            self.wof.manualTurn(1)
        elif self.gamempad2.getRawButton(6):
            self.wof.manualTurn(-1)
        else:
            self.wof.manualTurn(0)

        self.update_sd()

    def update_sd(self):
        self.sd.putNumber('Climb_Current_Draw', self.pdp.getCurrent(10))

        self.drive.update_smartdash()
        self.colorSensor.updateSD()
        self.wof.updateSD()
        self.vision.updateTable()

if __name__ == "__main__":
    wpilib.run(MyRobot)
