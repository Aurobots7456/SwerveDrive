import wpilib
import math

import ctre

from networktables import NetworkTables
from wpilib.controller import PIDController
from collections import namedtuple

ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

MAX_VOLTAGE = 5

class SwerveModule:
    driveMotor: ctre.WPI_VictorSPX
    rotateMotor: ctre.WPI_VictorSPX
        
    encoder: wpilib.AnalogInput

    cfg: ModuleConfig

    def setup(self):
        # Config
        self.sd_prefix = self.cfg.sd_prefix or 'Module'
        self.zero = self.cfg.zero or 0
        self.inverted = self.cfg.inverted or False
        self.allow_reverse = self.cfg.allow_reverse or True

        # SmartDashboard
        self.sd = NetworkTables.getTable('SmartDashboard')
        self.debugging = self.sd.getEntry('drive/drive/debugging')

        # Motor
        self.driveMotor.setInverted(self.inverted)

        self._requested_voltage = 0
        self._requested_speed = 0

        # Encoder
        self.encoder_zero = self.zero

        # PID Controller
        self._pid_controller = PIDController(1.5, 0.0, 0.0)
        self._pid_controller.enableContinuousInput(0.0, 5.0)

    def set_pid(self, p, i, d):
        self._pid_controller.setPID(p, i, d)

    def get_voltage(self):
        """
        :return: the voltage position after the zero
        """
        return self.encoder.getAverageVoltage() - self.encoder_zero

    def flush(self):
        """
        Flush the modules requested speed and voltage.
        """
        self._requested_voltage = self.encoder.getVoltage()
        self._requested_speed = 0

    @staticmethod
    def voltage_to_degrees(voltage):
        """
        Convert a given voltage value to degrees.

        :param voltage: a voltage value between 0 and 5
        """
        deg = (voltage / 5) * 360

        if deg < 0:
            deg += 360

        return deg

    @staticmethod
    def voltage_to_rad(voltage):
        """
        Convert a given voltage value to rad.

        :param voltage: a voltage value between 0 and 5
        """
        return (voltage / 5) * 2 * math.pi

    @staticmethod
    def degree_to_voltage(degree):
        """
        Convert a given degree to voltage.

        :param degree: a degree value between 0 and 360
        """
        return (degree / 360) * 5

    def zero_encoder(self):
        """
        Set the zero to the current voltage output.
        """
        self.encoder_zero = self.encoder.getVoltage()

    def is_aligned(self):
        """
        :return: Whether wheel is aligned to set point
        """
        return abs(self._pid_controller.getError()) < 0.1

    def _set_deg(self, value):
        """
        Round the value to within 360. Set the requested rotate position (requested voltage).
        Intended to be used only by the move function.
        """
        self._requested_voltage = ((self.degree_to_voltage(value) + self.encoder_zero) % 5)

    def move(self, speed, deg):
        """
        Set the requested speed and rotation of passed.

        :param speed: requested speed of wheel from -1 to 1
        :param deg: requested angle of wheel from 0 to 359 (Will wrap if over or under)
        """
        deg %= 360 # Prevent values past 360

        if self.allow_reverse:
            if abs(deg - self.voltage_to_degrees(self.get_voltage())) > 90:
                speed *= -1
                deg += 180
                deg %= 360

        self._requested_speed = speed
        self._set_deg(deg)

    def debug(self):
        """
        Print debugging information about the module to the log.
        """
        print(self.sd_prefix, '; requested_speed: ', self._requested_speed, ' requested_voltage: ', self._requested_voltage)

    def execute(self):
        """
        Use the PID controller to get closer to the requested position.
        Set the speed requested of the drive motor.

        Should be called every robot iteration/loop.
        """

        output = self._pid_controller.calculate(self.get_voltage(), self._requested_voltage)
        clampedOutput = max(min(output, 1), 0)
        self.rotateMotor.set(clampedOutput)

        self.driveMotor.set(self._requested_speed)

        self._requested_speed = 0.0

        self.update_smartdash()

    def update_smartdash(self):
        """
        Output a bunch on internal variables for debugging purposes.
        """
        self.sd.putNumber('drive/%s/degrees' % self.sd_prefix, self.voltage_to_degrees(self.get_voltage()))

        if self.debugging.getBoolean(False):
            self.sd.putNumber('drive/%s/requested_voltage' % self.sd_prefix, self._requested_voltage)
            self.sd.putNumber('drive/%s/requested_speed' % self.sd_prefix, self._requested_speed)
            self.sd.putNumber('drive/%s/raw voltage' % self.sd_prefix, self.encoder.getVoltage())  # DO NOT USE self.get_voltage() here
            self.sd.putNumber('drive/%s/average voltage' % self.sd_prefix, self.encoder.getAverageVoltage())
            self.sd.putNumber('drive/%s/encoder_zero' % self.sd_prefix, self.encoder_zero)

            self.sd.putNumber('drive/%s/PID' % self.sd_prefix, self._pid_controller.getSetpoint())
            self.sd.putNumber('drive/%s/PID Error' % self.sd_prefix, self._pid_controller.getPositionError())

            self.sd.putBoolean('drive/%s/allow_reverse' % self.sd_prefix, self.allow_reverse)
