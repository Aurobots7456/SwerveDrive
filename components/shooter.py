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

    def shoot(self):
        self.engage()

    def intake(self, speed):
        self.intake_speed = speed

    @timed_state(duration=1.0, first=True, next_state="feedShooter")
    def spinup(self):
        self.shooter_speed = 1

    @state
    def feedShooter(self):
        self.shooter_speed = 1
        self.belt_speed = 1

    def execute(self):
        super().execute()

        self.leftShooterMotor.set(self.shooter_speed)
        self.rightShooterMotor.set(-self.shooter_speed)
        self.intakeMotor.set(self.intake_speed)
        self.beltMotor.set(self.belt_speed)

        self.shooter_speed = 0
        self.intake_speed = 0
        self.belt_speed = 0
