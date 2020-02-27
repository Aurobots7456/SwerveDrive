import wpilib
import ctre

from magicbot import StateMachine, timed_state, state

from common import color_sensor

class WheelOfFortune(StateMachine):
    motor: ctre.WPI_VictorSPX

    color_sensor = color_sensor.REVColorSensor
    color_scheme = ['R', 'G', 'B', 'Y']

    target_color = ""

    def getData(self):
        match_data = wpilib.DriverStation.getGameSpecificMessage()

        if not match_data:
            self.target_color = match_data

    def calculate(self):
        index = self.color_scheme.index(self.target_color)
    
