import wpilib
import ctre

from magicbot import StateMachine, timed_state, state
from networktables import NetworkTables

from common import color_sensor

class WheelOfFortune(StateMachine):
    motor: ctre.WPI_VictorSPX
    colorSensor: color_sensor.ColorSensor

    def setup(self):
        self.sd = NetworkTables.getTable('SmartDashboard')

        self.color_scheme = ['R', 'G', 'B', 'Y']
        self.phase = 0

        self.target_color = "N"
        self.next_color = ""

        self.count = 0

        self.inProgress = False
        self.isCounted = False

    def getData(self):
        match_data = wpilib.DriverStation.getGameSpecificMessage()

        if not match_data == "":
            return match_data

        return "N"

    def manualTurn(self, speed):
        self.motor.set(speed)

    def reset(self):
        self.phase = 0
        self.target_color = "N"
        self.next_color = ""
        self.count = 0
        self.inProgress = False
        self.isCounted = False

    def handleFirstStage(self):
        self.phase = 1
        matched_color = self.colorSensor.matchColor()

        if not self.inProgress:
            if not matched_color == "N":
                self.target_color = matched_color

                index = self.color_scheme.index(matched_color)
                self.next_color = self.color_scheme[index - 1]
                self.inProgress = True

        if self.inProgress and not matched_color == "N":
            if count < 8:
                if matched_color == self.next_color:
                    self.isCounted = False
                
                if not self.isCounted:
                    if matched_color == self.target_color:
                        count += 1
                        self.isCounted = True

                self.motor.set(1)
            else:
                self.motor.set(0)
                self.reset()

    def handleSecondStage(self):
        self.phase = 2

        if not self.getData() == "N":
            matched_color = self.colorSensor.matchColor()

            if not self.inProgress:
                index = self.color_scheme.index(self.getData())
                index -= 2

                self.target_color = self.color_scheme[index]
                self.inProgress = True
            
            if self.inProgress and not matched_color == "N":
                if not matched_color == self.target_color:
                    self.motor.set(1)
                else:
                    self.motor.set(0)
                    self.reset()

    def updateSD(self):
        self.sd.putString("/wof/Target-Color", self.target_color)
        self.sd.putString("/wof/Next-Color", self.next_color)
        self.sd.putString("/wof/Game-Data", self.getData())
        self.sd.putNumber("/wof/phase", self.phase)
        self.sd.putNumber("/wof/count", self.count)
        self.sd.putBoolean("/wof/isCounted", self.isCounted)
        self.sd.putBoolean("/wof/inProgress", self.inProgress)
