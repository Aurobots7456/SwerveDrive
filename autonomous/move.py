from .base_auto import BaseAuto

from components import swervedrive

from magicbot.state_machine import state, timed_state, AutonomousStateMachine

class Middle(BaseAuto):

    MODE_NAME = "Middle"
    DEFAULT = True

    #Injection
    drive = swervedrive.SwerveDrive

    @timed_state(duration=5, next_state="finish", first=True)
    def drive_forward(self):
        self.drive.set_raw_fwd(0.25)

class Left(BaseAuto):

    MODE_NAME = "Left"
    DEFAULT = False

    #Injection
    drive = swervedrive.SwerveDrive

    @timed_state(duration=5, next_state="finish", first=True)
    def drive_left(self):
        self.drive.set_raw_strafe(0.25)

class Right(BaseAuto):

    MODE_NAME = "Right"
    DEFAULT = False

    #Injection
    drive = swervedrive.SwerveDrive

    @timed_state(duration=5, next_state="finish", first=True)
    def drive_left(self):
        self.drive.set_raw_strafe(-0.25)