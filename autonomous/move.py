from .base_auto import BaseAuto

from components import swervedrive, shooter
from common import vision

from magicbot.state_machine import state, timed_state, AutonomousStateMachine

class Middle(BaseAuto):
    MODE_NAME = "Middle"
    DEFAULT = True

    drive: swervedrive.SwerveDrive
    shooter: shooter.Shooter
    vision = vision.Vision()

    @timed_state(duration=10, next_state="shoot", first=True)
    def vision_align(self):
        drive = self.vision.verticalAdjust()
        rotate = self.vision.horizontalAdjust()

        if drive > 0.1 or drive < -0.1:
            self.drive.set_raw_fwd(drive * 0.35)
        else:
            self.drive.set_raw_fwd(0)
            if rotate > 0.1 or rotate < -0.1:
                self.drive.set_raw_rcw(rotate * 0.35)
            else:
                self.drive.set_raw_rcw(0)

        self.drive.update_smartdash()

    @timed_state(duration=5, next_state="finish")
    def shoot(self):
        self.shooter.shoot()

class Left(BaseAuto):
    MODE_NAME = "Left"
    DEFAULT = False

    drive: swervedrive.SwerveDrive

    @timed_state(duration=5, next_state="finish", first=True)
    def drive_left(self):
        self.drive.set_raw_strafe(0.25)

class Right(BaseAuto):
    MODE_NAME = "Right"
    DEFAULT = False

    drive: swervedrive.SwerveDrive

    @timed_state(duration=5, next_state="right", first=True)
    def drive_left(self):
        self.drive.set_raw_strafe(-0.25)