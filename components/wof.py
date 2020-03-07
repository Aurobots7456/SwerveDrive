import wpilib
import ctre

from magicbot import StateMachine, timed_state, state
from networktables import NetworkTables

from common import color_sensor

class WheelOfFortune():
    # Get the motors from the injection
    motor: ctre.WPI_VictorSPX
    colorSensor: color_sensor.ColorSensor

    def setup(self):
        # Get the table
        self.sd = NetworkTables.getTable('SmartDashboard')

        # Array of the colors respected to the order on the wheel 
        self.color_scheme = ['R', 'G', 'B', 'Y']

        # Phase indicator (0 = N/A, 1 = Spin, 2 = Color)
        self.phase = 0

        # Variables to be used by the handlers
        self.target_color = "N"
        self.next_color = ""
        self.count = 0
        self.isCounted = False # Used to prevent counting the same piece multiple times
        self.inProgress = False # Indicates that a function is already running

    def getData(self):
        '''
        Get the color from the driver station.
        If the color is not released yet, returns "N".

        :return: First letter of the color (R / G / B / Y) or N for none.
        '''
        # Get the game specific message from the driver station
        match_data = wpilib.DriverStation.getGameSpecificMessage(wpilib.DriverStation.getInstance())

        if not match_data == "":
            # If data exist return the data
            return match_data

        # If data does not exist return "N"
        return "N"

    def manualTurn(self, speed):
        '''
        Manualy set the speed of the WoF motor.
        :param speed: [-1, 1]
        '''
        self.motor.set(speed)

    def reset(self):
        '''
        Reset all the variables to the initial state.
        Unless you intend to change the phase or restart the current phase,
        do not use this function.
        '''
        self.phase = 0
        self.target_color = "N"
        self.next_color = ""
        self.count = 0
        self.inProgress = False
        self.isCounted = False

    def handleFirstStage(self):
        '''
        Automatically complete the first wheel of fortune objective (spin it 3 to 5 turns).
        Once started, the function will get the current color and spin the wheel until
        it sees the color 8 more times.
        '''
        self.phase = 1 # Set the phase to 1
        matched_color = self.colorSensor.matchColor() # Get the current color

        # Initialization
        if not self.inProgress:
            if not matched_color == "N":
                # If the color exists, set it as the target
                self.target_color = matched_color

                # The color after will be set as the next color
                index = self.color_scheme.index(matched_color)
                self.next_color = self.color_scheme[index - 1] # -1 because how the array is ordered
                self.inProgress = True

        # Periodic
        if self.inProgress and not matched_color == "N":
            if self.count < 8: # Until the count reaches 8 (3,5 turns to be safe)
                if matched_color == self.next_color:
                    # If the next color is read reset isCounted back to false
                    self.isCounted = False
                
                if not self.isCounted:
                    # If it is not counted yet, count 1 when reached to the target color
                    if matched_color == self.target_color:
                        self.count += 1
                        self.isCounted = True

                self.motor.set(1)
            else:
                # When reached the desired amount of turns stop and reset.
                self.motor.set(0)
                # self.reset()

    def handleSecondStage(self):
        '''
        Automatically complete the second wheel of fortune objective (spin it to a specific color).
        Once started, the function will get the objective color from the FMS and spin the wheel until
        that color is under the field's color sensor.
        '''
        self.phase = 2 # Set the phase to 2

        if not self.getData() == "N": # If the game data is released
            matched_color = self.colorSensor.matchColor() # Get the current color

            # Initialization
            if not self.inProgress:
                # Get the two previous color and set it as the target color
                index = self.color_scheme.index(self.getData())
                index -= 2

                self.target_color = self.color_scheme[index]
                self.inProgress = True
            
            # Periodic
            if self.inProgress and not matched_color == "N":
                # Spin the wheel until it sees the target color
                if not matched_color == self.target_color:
                    self.motor.set(0.35)
                else:
                    self.motor.set(0)
                    self.reset()

    def execute(self):
        return

    def updateSD(self):
        # Update the SmartDashboard
        self.sd.putString("/wof/Target-Color", self.target_color)
        self.sd.putString("/wof/Next-Color", self.next_color)
        self.sd.putString("/wof/Game-Data", self.getData())
        self.sd.putNumber("/wof/phase", self.phase)
        self.sd.putNumber("/wof/count", self.count)
        self.sd.putBoolean("/wof/isCounted", self.isCounted)
        self.sd.putBoolean("/wof/inProgress", self.inProgress)
