import wpilib
import ctre

from magicbot import StateMachine, timed_state, state

from components import swervedrive
from common import vision

class Shooter(StateMachine):
    """
    Shooter is a StateMachine which means every iteration (10ms),
    it will call the self.execute function. In order to move any part,
    Speeds should be changed before the execution.

    @timed_states will run everything inside them and after the duration,
    they will pass to the next function. 
    """
    # Get the motors, drive and vision from injection
    drive: swervedrive.SwerveDrive

    leftShooterMotor: ctre.WPI_VictorSPX
    rightShooterMotor: ctre.WPI_VictorSPX
    beltMotor: ctre.WPI_VictorSPX
    intakeMotor: ctre.WPI_VictorSPX

    vision: vision.Vision

    # Set the speeds to zero.
    shooter_speed = 0
    intake_speed = 0
    belt_speed = 0

    def stop(self):
        """
        Set every motor to 0 and finish the iteration.
        """
        self.shooter_speed = 0
        self.intake_speed = 0
        self.belt_speed = 0

        self.done()

    def unload(self):
        """
        Set the motors reverse to unload the ball.
        """
        self.shooter_speed = -0.3
        self.intake_speed = -0.5
        self.belt_speed = -0.5

    def intake(self):
        """
        Set the intake and the belt motors to take the ball.
        """
        self.intake_speed = 0.57
        self.belt_speed = 0.5

    def align(self):
        """
        Using the limelight, autonomously align the robot.
        """
        # Get the adjustment values from the vision component.
        drive = self.vision.verticalAdjust()
        rotate = self.vision.horizontalAdjust()

        # Allign the robot.
        if not drive == 0:
            self.drive.set_raw_fwd(drive * 0.35)
        else:
            self.drive.set_raw_fwd(0)
            if not rotate == 0:
                self.drive.set_raw_rcw(rotate * 0.35)
            else:
                self.drive.set_raw_rcw(0)

    def shoot(self):
        """
        This will start the shooting procedure by calling the @timed_states.
        """
        self.engage()

    # The adjust function will not run (disabled) because the first state is spinup.
    # If you want to activate the adjust remove "first=True" from the spinup
    # and put it to the adjust.

    @timed_state(duration=0.25, next_state="spinup")
    def adjust(self):
        """
        Unload the ball for 0.25 seconds, then call spinup.
        """
        self.unload()

    @timed_state(duration=0.5, first=True, next_state="feedShooter")
    def spinup(self):
        """
        Spinup the shooters for 0.5 seconds the call the feedShooter.
        """
        self.shooter_speed = 0.7

    @state
    def feedShooter(self):
        """
        Until the stop function is called feed the shooter.
        """
        self.shooter_speed = 0.7
        self.belt_speed = 0.5

    def execute(self):
        """
        Execute the component using the preset speeds.
        """
        super().execute()

        self.leftShooterMotor.set(self.shooter_speed)
        self.rightShooterMotor.set(-self.shooter_speed)
        self.intakeMotor.set(self.intake_speed)
        self.beltMotor.set(self.belt_speed)

        # Reset all the speeds to 0
        self.shooter_speed = 0
        self.intake_speed = 0
        self.belt_speed = 0
