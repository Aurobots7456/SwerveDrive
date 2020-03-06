import math

from magicbot import magiccomponent
from components import swervemodule

from networktables import NetworkTables
from networktables.util import ntproperty

class SwerveDrive:
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule

    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.1)
    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 0.5)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 0.65)
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', False)

    def setup(self):
        """
        Called after injection.
        """
        # Put all the modules into a dictionary
        self.modules = {
            'front_left': self.frontLeftModule,
            'front_right': self.frontRightModule,
            'rear_left': self.rearLeftModule,
            'rear_right': self.rearRightModule
        }

        # Get Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')

        # Set all inputs to zero
        self._requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self._requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self._requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        # Variables that allow enabling and disabling of features in code
        self.squared_inputs = True
        self.threshold_input_vectors = True

        self.width = (19.5 / 12) / 2 # (Inch / 12 = Foot) / 2
        self.length = (22 / 12) / 2 # (Inch / 12 = Foot) / 2

        self.request_wheel_lock = False

    @property
    def chassis_dimension(self):
        return (self.width, self.length)

    @chassis_dimension.setter
    def chassis_dimension(self, dimension):
        self.width = dimension[0]
        self.length = dimension[1]

    @staticmethod
    def square_input(input):
        return math.copysign(input * input, input) # Return magnitude of x but the sign of y. (x, y)
    
    @staticmethod
    def normalize(data):
        maxMagnitude = max(abs(x) for x in data)

        if maxMagnitude > 1.0:
            for i in range(len(data)):
                data[i] = data[i] / maxMagnitude
        
        return data

    @staticmethod
    def normalizeDictionary(data):
        maxMagnitude = max(abs(x) for x in data.values())

        if maxMagnitude > 1.0:
            for key in data:
                data[key] = data[key] / maxMagnitude
        
        return data

    def flush(self):
        """
        This method should be called to reset all requested values of the drive system.
        """
        self._requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self._requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self._requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        for module in self.modules.values():
            module.flush()

    def set_raw_fwd(self, fwd):
        """
        Sets the raw fwd value to prevent it from being passed through any filters

        :param fwd: A value from -1 to 1
        """
        self._requested_vectors['fwd'] = fwd

    def set_raw_strafe(self, strafe):
        """
        Sets the raw strafe value to prevent it from being passed through any filters

        :param strafe: A value from -1 to 1
        """
        self._requested_vectors['strafe'] = strafe
    
    def set_raw_rcw(self, rcw):
        """
        Sets the raw rcw value to prevent it from being passed through any filters

        :param rcw: A value from -1 to 1
        """
        self._requested_vectors['rcw'] = rcw

    def set_fwd(self, fwd):
        """
        Individually sets the fwd value. (passed through filters)

        :param fwd: A value from -1 to 1
        """
        if self.squared_inputs:
            fwd = self.square_input(fwd)

        fwd *= self.xy_multiplier

        self._requested_vectors['fwd'] = fwd

    def set_strafe(self, strafe):
        """
        Individually sets the strafe value. (passed through filters)

        :param strafe: A value from -1 to 1
        """
        if self.squared_inputs:
            strafe = self.square_input(strafe)

        strafe *= self.xy_multiplier

        self._requested_vectors['strafe'] = strafe

    def set_rcw(self, rcw):
        """
        Individually sets the rcw value. (passed through filters)

        :param rcw: A value from -1 to 1
        """
        if self.squared_inputs:
            rcw = self.square_input(rcw)

        rcw *= self.rotation_multiplier

        self._requested_vectors['rcw'] = rcw

    def move(self, fwd, strafe, rcw):
        """
        Calulates the speed and angle for each wheel given the requested movement

        Positive fwd value = Forward robot movement
        Negative fwd value = Backward robot movement
        Positive strafe value = Left robot movement
        Negative strafe value = Right robot movement

        :param fwd: the requested movement in the Y direction 2D plane
        :param strafe: the requested movement in the X direction of the 2D plane
        :param rcw: the requestest magnatude of the rotational vector of a 2D plane
        """
        self.set_fwd(fwd)
        self.set_strafe(strafe)
        self.set_rcw(rcw)

    def _calculate_vectors(self):
        """
        Calculate the requested speed and angle of each modules from self._requested_vectors and store them in
        self._requested_speeds and self._requested_angles dictionaries.
        """
        self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw'] = self.normalize([self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw']])

        # Does nothing if the values are lower than the input thresh
        if self.threshold_input_vectors:
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
                self._requested_vectors['fwd'] = 0

            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                self._requested_vectors['strafe'] = 0

            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                self._requested_vectors['rcw'] = 0

            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0:  # Prevents a useless loop.
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0) # Do NOT reset the wheel angles.

                if self.request_wheel_lock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defence)

                    self._requested_angles['front_left'] = 45
                    self._requested_angles['front_right'] = -45
                    self._requested_angles['rear_left'] = -45
                    self._requested_angles['rear_right'] = 45

                    self.request_wheel_lock = False

                return
        
        ratio = math.hypot(self.length, self.width)

        # Velocities per quadrant
        frontX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (self.length / ratio))
        rearX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (self.length / ratio))
        leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (self.width / ratio))
        rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (self.width / ratio))

        # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        frontLeft_speed = math.hypot(frontX, rightY)
        frontLeft_angle = math.degrees(math.atan2(frontX, rightY))

        frontRight_speed = math.hypot(frontX, leftY)
        frontRight_angle = math.degrees(math.atan2(frontX, leftY))

        rearLeft_speed = math.hypot(rearX, rightY)
        rearLeft_angle = math.degrees(math.atan2(rearX, rightY))

        rearRight_speed = math.hypot(rearX, leftY)
        rearRight_angle = math.degrees(math.atan2(rearX, leftY))

        self._requested_speeds['front_left'] = frontLeft_speed
        self._requested_speeds['front_right'] = frontRight_speed
        self._requested_speeds['rear_left'] = rearLeft_speed
        self._requested_speeds['rear_right'] = rearRight_speed

        self._requested_angles['front_left'] = frontLeft_angle
        self._requested_angles['front_right'] = frontRight_angle
        self._requested_angles['rear_left'] = rearLeft_angle
        self._requested_angles['rear_right'] = rearRight_angle

        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

        # Zero request vectors for saftey reasons
        self._requested_vectors['fwd'] = 0.0
        self._requested_vectors['strafe'] = 0.0
        self._requested_vectors['rcw'] = 0.0

    def debug(self, debug_modules=False):
        """
        Prints debugging information to log
        """
        if debug_modules:
            for key in self.modules:
                self.modules[key].debug()
        
        print('Requested values: ', self._requested_vectors, '\n')
        print('Requested angles: ', self._requested_angles, '\n')
        print('Requested speeds: ', self._requested_speeds, '\n')

    def execute(self):
        """
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        """
        self.update_smartdash()

        self._calculate_vectors()

        for key in self.modules:
            self.modules[key].move(self._requested_speeds[key], self._requested_angles[key])

        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)

        for key in self.modules:
            self.modules[key].execute()
        
    def update_smartdash(self):
        """
        Pushes some internal variables for debugging.
        """
        if self.debugging:
            for key in self._requested_angles:
                self.sd.putNumber('drive/drive/%s_angle' % key, self._requested_angles[key])
                self.sd.putNumber('drive/drive/%s_speed' % key, self._requested_speeds[key])
