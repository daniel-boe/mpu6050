"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer

Forked by Daniel Boe
"""
import smbus2
import logging
from math import acos,sqrt,pi

l = logging.getLogger(__name__)
l.setLevel(logging.root.level)

class mpu6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0
    
    ACCEL_SCALE_MAP={'2g':ACCEL_SCALE_MODIFIER_2G,
                     '4g':ACCEL_SCALE_MODIFIER_4G,
                     '8g':ACCEL_SCALE_MODIFIER_8G,
                     '16g':ACCEL_SCALE_MODIFIER_16G}
        
    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    GYRO_SCALE_MAP={'250deg':GYRO_SCALE_MODIFIER_250DEG,
                    '500deg':GYRO_SCALE_MODIFIER_500DEG,
                    '1000deg':GYRO_SCALE_MODIFIER_1000DEG,
                    '2000deg':GYRO_SCALE_MODIFIER_2000DEG}

    # Pre-defined ranges (Page 15 in manual)
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    ACCEL_RANGE_MAP={'2g':ACCEL_RANGE_2G,
                     '4g':ACCEL_RANGE_4G,
                     '8g':ACCEL_RANGE_8G,
                     '16g':ACCEL_RANGE_16G}


    # Pre-defined ranges (Page 14 in manual)
    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    GYRO_RANGE_MAP={'250deg':GYRO_RANGE_250DEG,
                    '500deg':GYRO_RANGE_500DEG,
                    '1000deg':GYRO_RANGE_1000DEG,
                    '2000deg':GYRO_RANGE_2000DEG}

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    # Accelerometer registers
    ACCEL_XOUTH = 0x3B
    ACCEL_XOUTL = 0x3C
    ACCEL_YOUTH = 0x3D
    ACCEL_YOUTL = 0x3E
    ACCEL_ZOUTH = 0x3F
    ACCEL_ZOUTL = 0x40

    # Temperature Registers
    TEMP_OUTH = 0x41
    TEMP_OUTL = 0x41

    # Gyro Registers
    GYRO_XOUTH = 0x43
    GYRO_XOUTL = 0x44
    GYRO_YOUTH = 0x45
    GYRO_YOUTL = 0x46
    GYRO_ZOUTH = 0x47
    GYRO_ZOUTL = 0x48

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        
        l.info(f'{__class__.__name__}.__init__(), address={address}')
        self.address = address
        self.bus = smbus2.SMBus(bus)

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        self.set_accel_range('4g')
        self.set_gyro_range('500deg')

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        try:
            high = self.bus.read_byte_data(self.address, register)
        except OSError:
            l.error(f'{__class__.__name__}.read_i2c_word(), register = {register}, IO Error, reading first 8 bits')
            return None
        try:
            low = self.bus.read_byte_data(self.address, register + 1)
        except OSError:
            l.error(f'{__class__.__name__}.read_i2c_word(), register = {register}, IO Error, reading second 8 bits')
            return None

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """

        l.info(f'{__class__.__name__}.get_temp()')

        raw_temp = self.read_i2c_word(self.TEMP_OUTH)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """

        l.info(f'{__class__.__name__}.set_accel_range() - Setting acceleration Range to {accel_range}')
        if mpu6050.ACCEL_RANGE_MAP.get(accel_range) is None:
            raise ValueError(f'{accel_range} is not a valid Acceleration Range')

        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, self.ACCEL_RANGE_MAP.get(accel_range))
        
        # Store the current acceleration range
        self.accel_range=accel_range

    def get_accel_data(self):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        l.info(f'{__class__.__name__}.get_accel_data()')

        xyz=[self.read_i2c_word(reg) for reg in [mpu6050.ACCEL_XOUTH,mpu6050.ACCEL_YOUTH,mpu6050.ACCEL_ZOUTH]]
            
        try:
            xyz=[i/mpu6050.ACCEL_SCALE_MAP.get(self.accel_range)*mpu6050.GRAVITIY_MS2 for i in xyz]
        except TypeError:
            l.error(f'{__class__.__name__}.get_accel_data(), error')         
            xyz=[None,None,None]

        return {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]}

    def calculate_angle(self,xyz):
        """
           Calculates the angles between the acceleration vector components
        """
        l.info(f'{__class__.__name__}.calculate_angle()')
        try:
            angle=acos(xyz['y']/sqrt(xyz['x']**2 + xyz['y']**2 + xyz['z']**2))*180/pi
        except (ValueError,TypeError):
            angle=None
        return angle 

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.

        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        l.info(f'{__class__.__name__}.set_gyro_range() - Setting gyro Range to {gyro_range}')
        if mpu6050.GYRO_RANGE_MAP.get(gyro_range) is None:
            raise ValueError(f'{gyro_range} is not a valid Acceleration Range')

        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, mpu6050.GYRO_RANGE_MAP.get(gyro_range))

        # Set the current gyro range
        self.gyro_range=gyro_range

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        l.info(f'{__class__.__name__}.get_gyro_data()')
        xyz=[self.read_i2c_word(reg) for reg in [mpu6050.GYRO_XOUTH,mpu6050.GYRO_YOUTH,mpu6050.GYRO_ZOUTH]]

        try:
            xyz=[i/mpu6050.GYRO_SCALE_MAP.get(self.gyro_range) for i in xyz]
        except TypeError:
            l.error(f'{__class__.__name__}.get_gyro_data(), error')         
            xyz=[None,None,None]

        return {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]}
