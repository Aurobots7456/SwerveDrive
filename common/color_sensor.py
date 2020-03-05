from wpilib import I2C, Color
from rev.color import ColorSensorV3, ColorMatch
from networktables import NetworkTables

class ColorSensor():
    def __init__(self):
        self.sd = NetworkTables.getTable('SmartDashboard')

        self.colorSensor = ColorSensorV3(I2C.Port.kOnboard)
        self.colorMatcher = ColorMatch()

        self.kBlue = Color(0.163, 0.444, 0.390)
        self.kGreen = Color(0.164, 0.548, 0.255)
        self.kRed = Color(0.561, 0.232, 0.114)
        self.kYellow = Color(0.361, 0.524, 0.113)
        
        self.colorMatcher.addColorMatch(self.kBlue)
        self.colorMatcher.addColorMatch(self.kGreen)
        self.colorMatcher.addColorMatch(self.kRed)
        self.colorMatcher.addColorMatch(self.kYellow)

    def getColor(self):
        return self.colorSensor.getColor()

    def matchColor(self):
        color = self.getColor()
        self.match = self.colorMatcher.matchClosestColor(color, 0.9)
        # print(color)

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
