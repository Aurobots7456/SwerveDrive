from .base_auto import BaseAuto

from magicbot.state_machine import state, timed_state, AutonomousStateMachine

from components import swervedrive, shooter
from common import vision

"""
These are the autonomous codes for the robot.

Currently, there are 3 modes:
    Default: Will align and then shoot
    OnlyShoot: Will shoot
    OnlyMove: Will move

In order to switch modes, use the DS.
"""

class Default(BaseAuto):
    MODE_NAME = "Default"
    DEFAULT = True # If no mode is selected this will be the default.

    # Injection
    drive: swervedrive.SwerveDrive
    shooter: shooter.Shooter
    vision: vision.Vision

    # For 7 seconds try to align.
    # If aligned or 7 seconds past shoot.
    @timed_state(duration=7, first=True, next_state="shoot")
    def vision_align(self):
        drive = self.vision.verticalAdjust()
        rotate = self.vision.horizontalAdjust()

        if not drive == 0:
            self.drive.set_raw_fwd(drive * 0.35)
        else:
            self.drive.set_raw_fwd(0)
            if not rotate == 0:
                self.drive.set_raw_rcw(rotate * 0.35)
            else:
                self.drive.set_raw_rcw(0)
                self.next_state('shoot')

        self.drive.update_smartdash()

    # Shoot
    @timed_state(duration=4, next_state="escape")
    def shoot(self):
        self.shooter.shoot()

    # Drive backwards.
    @timed_state(duration=2.5, next_state="finish")
    def escape(self):
        self.shooter.stop()
        self.drive.set_raw_fwd(0.5)

class OnlyShoot(BaseAuto):
    MODE_NAME = "Only Shoot"
    DEFAULT = False

    # Injection
    drive: swervedrive.SwerveDrive

    # Shoot
    @timed_state(duration=4, first=True, next_state="escape")
    def shoot(self):
        self.shooter.shoot()
    
    # Drive backwards.
    @timed_state(duration=2.5, next_state="finish")
    def escape(self):
        self.shooter.stop()
        self.drive.set_raw_fwd(0.5)

class OnlyMove(BaseAuto):
    MODE_NAME = "Only Move"
    DEFAULT = False

    # Injection
    drive: swervedrive.SwerveDrive

    # Drive backwards.
    @timed_state(duration=2.5, next_state="finish", first=True)
    def drive(self):
        self.drive.set_raw_fwd(0.35)