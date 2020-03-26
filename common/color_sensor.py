from wpilib import I2C, Color
from rev.color import ColorSensorV3, ColorMatch
from networktables import NetworkTables

class ColorSensor():
    def __init__(self):
        self.sd = NetworkTables.getTable('SmartDashboard')

        self.colorSensor = ColorSensorV3(I2C.Port.kOnboard)
        self.colorMatcher = ColorMatch()

        # Create possible colors to match from
        # These values need to change for different environments.
        # Use the print(color) line in the mathColor function to see the color values.
        # Get the values from the driver station's console and adjust these values.
        self.kBlue = Color(0.167, 0.446, 0.385)
        self.kGreen = Color(0.202, 0.535, 0.261)
        self.kRed = Color(0.395, 0.409, 0.195)
        self.kYellow = Color(0.302, 0.542, 0.155)
        
        # Add the colors to the matcher as options
        self.colorMatcher.addColorMatch(self.kBlue)
        self.colorMatcher.addColorMatch(self.kGreen)
        self.colorMatcher.addColorMatch(self.kRed)
        self.colorMatcher.addColorMatch(self.kYellow)

    def getColor(self):
        '''
        Get the current color from the sensor in raw format.
        :returns: frc::Color class with normalized sRGB values
        '''
        return self.colorSensor.getColor()

    def matchColor(self):
        '''
        Match the raw color output from the sensor with an option.
        :returns: First letter of the color (R / G / B / Y) or N for none.
        '''
        color = self.getColor()
        # Match the color with 90% of confidence
        self.match = self.colorMatcher.matchClosestColor(color, 0.9)
        # TODO: Adjust color values using the print function.
        # print(color)
        # If you don't see the prints in the DS, press the config button (gear)
        # on the top left of the consol and select "+ Prints" option.

        # Return the first letter of the color that matches the sensor value.
        if (self.match == self.kBlue):
            return "B"
        elif (self.match == self.kGreen):
            return "G"
        elif (self.match == self.kRed):
            return "R"
        elif (self.match == self.kYellow):
            return "Y"
        else:
            return "N"

    def updateSD(self):
        self.sd.putString("/wof/Color Match", self.matchColor())
