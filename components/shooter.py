import wpilib
import ctre

from magicbot import StateMachine, timed_state, state

class Shooter(StateMachine):
    # Motors
    shooterMotor = ctre.WPI_VictorSPX
    beltMotor = ctre.WPI_VictorSPX

    # Motor speeds
    beltSpeed = 0
    shooterSpeed = 0

    def stop(self):
        """
        This method is called to stop the shooter and complete the state. 
        """
        beltSpeed = 0
        shooterSpeed = 0

        self.done()

    def shoot(self):
        """
        This method is called to initiate the shooter.
        """
        self.engage()

    @timed_state(duration=1.5, first=True, next_state="feedShooter")
    def spinup(self):
        """
        Spin the shooter before feeding it (1.5 second delay).
        """
        self.shooterSpeed = 1

    @state
    def feedShooter(self):
        """
        Feed the shooter.
        """
        self.beltSpeed = 1

    def execute(self):
        super().execute()

        self.beltMotor.set(self.beltSpeed)
        self.shooterMotor.set(self.shooterSpeed)

        self.beltSpeed = 0
        self.shooterSpeed = 0
