import wpilib
import ctre

from magicbot import StateMachine, timed_state, state
from components import swervedrive
from common import vision

class Shooter(StateMachine):
    drive: swervedrive.SwerveDrive

    leftShooterMotor: ctre.WPI_VictorSPX
    rightShooterMotor: ctre.WPI_VictorSPX
    beltMotor: ctre.WPI_VictorSPX
    intakeMotor: ctre.WPI_VictorSPX

    vision: vision.Vision

    shooter_speed = 0
    intake_speed = 0
    belt_speed = 0

    def stop(self):
        self.shooter_speed = 0
        self.intake_speed = 0
        self.belt_speed = 0

        self.done()

    def unload(self):
        self.shooter_speed = -0.3
        self.intake_speed = -0.5
        self.belt_speed = -0.5

    def intake(self):
        self.intake_speed = 0.57
        self.belt_speed = 0.5

    def align(self):
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
                self.shoot()

    def shoot(self):
        self.engage()

    @timed_state(duration=0.25, first=True, next_state="spinup")
    def adjust(self):
        self.unload()

    @timed_state(duration=0.5, next_state="feedShooter")
    def spinup(self):
        self.shooter_speed = 0.8

    @state
    def feedShooter(self):
        self.shooter_speed = 0.8
        self.belt_speed = 0.5

    def execute(self):
        super().execute()

        self.leftShooterMotor.set(self.shooter_speed)
        self.rightShooterMotor.set(-self.shooter_speed)
        self.intakeMotor.set(self.intake_speed)
        self.beltMotor.set(self.belt_speed)

        self.shooter_speed = 0
        self.intake_speed = 0
        self.belt_speed = 0
