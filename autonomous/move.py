from .base_auto import BaseAuto

from components import swervedrive, shooter
from common import vision

from magicbot.state_machine import state, timed_state, AutonomousStateMachine

class Default(BaseAuto):
    MODE_NAME = "Default"
    DEFAULT = True

    drive: swervedrive.SwerveDrive
    shooter: shooter.Shooter
    vision: vision.Vision

    @timed_state(duration=10, first=True)
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
                self.next_state('shoot')

        self.drive.update_smartdash()

    @timed_state(duration=5, next_state="finish")
    def shoot(self):
        self.shooter.shoot()

class OnlyMove(BaseAuto):
    MODE_NAME = "Only Move"
    DEFAULT = False

    drive: swervedrive.SwerveDrive

    @timed_state(duration=3, next_state="finish", first=True)
    def drive(self):
        self.drive.set_raw_fwd(0.35)