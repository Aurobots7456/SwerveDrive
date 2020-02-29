import wpilib
import ctre

from magicbot import StateMachine, timed_state, state

class Shooter(StateMachine):
    leftShooterMotor: ctre.WPI_VictorSPX
    rightShooterMotor: ctre.WPI_VictorSPX
    beltMotor: ctre.WPI_VictorSPX
    intakeMotor: ctre.WPI_VictorSPX

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

    def intake(self, speed):
        self.intake_speed = speed

    def shoot(self):
        self.engage()

    @timed_state(duration=0.5, first=True, next_state="spinup")
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
