import wpilib
import ctre

from magicbot import StateMachine, timed_state, state

class Shooter(StateMachine):
    shooterMotor = ctre.WPI_VictorSPX
    beltMotor = ctre.WPI_VictorSPX

    beltSpeed = 0
    shooterSpeed = 0

    def stop(self):
        beltSpeed = 0
        shooterSpeed = 0

        self.done()

    def shoot(self):
        self.engage()

    @timed_state(duration=1.5, first=True, next_state="feedShooter")
    def spinup(self):
        self.shooterSpeed = 1

    @state
    def feedShooter(self):
        self.beltSpeed = 1

    def execute(self):
        super().execute()

        self.beltMotor.set(self.beltSpeed)
        self.shooterMotor.set(self.shooterSpeed)

        self.beltSpeed = 0
        self.shooterSpeed = 0
