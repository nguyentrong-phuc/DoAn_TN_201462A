from __future__ import division
import logging
import time
from time import sleep
import math

#MPU6050 Registers/etc:
MPU6050_ADDRESS     = 0x68
INT_pin             = 0x07
SAMPLE_RATE         = 0x19
CONFIG              = 0x1A
GYRO_CONFIG         = 0x1B
ACC_CONFIG          = 0x1C
INT_ENABLE          = 0x38    
PWR_MGMT_1          = 0x6B
ACC_XOUT_H	        = 0x3B
ACC_YOUT_H	        = 0x3D
ACC_ZOUT_H	        = 0x3F
GYRO_XOUT_H         = 0x43
GYRO_YOUT_H         = 0x45
GYRO_ZOUT_H         = 0x47
TEMP_OUT            = 0x41

#PCA9685 Registers/etc:
PCA9685_ADDRESS    = 0x40
MODE1              = 0x00
MODE2              = 0x01
SUBADR1            = 0x02
SUBADR2            = 0x03
SUBADR3            = 0x04
PRESCALE           = 0xFE
LED0_ON_L          = 0x06
LED0_ON_H          = 0x07
LED0_OFF_L         = 0x08
LED0_OFF_H         = 0x09
ALL_LED_ON_L       = 0xFA
ALL_LED_ON_H       = 0xFB
ALL_LED_OFF_L      = 0xFC
ALL_LED_OFF_H      = 0xFD

# Bits:
RESTART            = 0x80
SLEEP              = 0x10
ALLCALL            = 0x01
INVRT              = 0x10
OUTDRV             = 0x04


logger = logging.getLogger(__name__)


def software_reset(i2c=None, **kwargs):
    """Sends a software reset (SWRST) command to all servo drivers on the bus."""
    # Setup I2C interface for device 0x00 to talk to all of them.
    if i2c is None:
        import Adafruit_GPIO.I2C as I2C
        i2c = I2C
    self._device = i2c.get_i2c_device(0x00, **kwargs)
    self._device.writeRaw8(0x06)  # SWRST


class PCA9685(object):    # pwm = PCA9685()
    """PCA9685 PWM LED/servo controller."""

    def __init__(self, address=PCA9685_ADDRESS, i2c=None, **kwargs):
        """Initialize the PCA9685."""
        # Setup I2C interface for the device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
      
        self.set_all_pwm(0, 0)
        self._device.write8(MODE2, OUTDRV)
        self._device.write8(MODE1, ALLCALL)
        time.sleep(0.005)  # wait for oscillator
        mode1 = self._device.readU8(MODE1)
        mode1 = mode1 & ~SLEEP  # wake up (reset sleep)
        self._device.write8(MODE1, mode1)
        time.sleep(0.005)  # wait for oscillator

    def set_pwm_freq(self, freq_hz):
        """Set the PWM frequency to the provided value in hertz."""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        logger.debug('Setting PWM frequency to {0} Hz'.format(freq_hz))
        logger.debug('Estimated pre-scale: {0}'.format(prescaleval))
        prescale = int(math.floor(prescaleval + 0.5))
        logger.debug('Final pre-scale: {0}'.format(prescale))
        oldmode = self._device.readU8(MODE1);
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self._device.write8(MODE1, newmode)  # go to sleep
        self._device.write8(PRESCALE, prescale)
        self._device.write8(MODE1, oldmode)
        time.sleep(0.005)
        self._device.write8(MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        """Sets a single PWM channel."""
        self._device.write8(LED0_ON_L+4*channel, on & 0xFF)
        self._device.write8(LED0_ON_H+4*channel, on >> 8)
        self._device.write8(LED0_OFF_L+4*channel, off & 0xFF)
        self._device.write8(LED0_OFF_H+4*channel, off >> 8)

    def set_pwm_servo(self, channel, on, off1,off2, off3):
        """Sets a single PWM channel."""
        self._device.write8(LED0_ON_L+4*channel, on & 0xFF)
        self._device.write8(LED0_ON_H+4*channel, on >> 8)
        self._device.write8(LED0_OFF_L+4*channel, off1 & 0xFF)
        self._device.write8(LED0_OFF_H+4*channel, off1 >> 8)
        #
        self._device.write8(LED0_ON_L+4*(channel+1), on & 0xFF)
        self._device.write8(LED0_ON_H+4*(channel+1), on >> 8)
        self._device.write8(LED0_OFF_L+4*(channel+1), off2 & 0xFF)
        self._device.write8(LED0_OFF_H+4*(channel+1), off2 >> 8)
        #
        self._device.write8(LED0_ON_L+4*(channel+2), on & 0xFF)
        self._device.write8(LED0_ON_H+4*(channel+2), on >> 8)
        self._device.write8(LED0_OFF_L+4*(channel+2), off3 & 0xFF)
        self._device.write8(LED0_OFF_H+4*(channel+2), off3 >> 8)
        
    def set_all_pwm(self, on, off):
        """Sets all PWM channels."""
        self._device.write8(ALL_LED_ON_L, on & 0xFF)
        self._device.write8(ALL_LED_ON_H, on >> 8)
        self._device.write8(ALL_LED_OFF_L, off & 0xFF)
        self._device.write8(ALL_LED_OFF_H, off >> 8)


class MPU6050(object): # mpu = MPU6050()
    def __init__(self, address=MPU6050_ADDRESS, i2c=None, **kwargs):
        """Initialize the PCA9685."""
        # Setup I2C interface for the device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)
        
        self._device.write8(SAMPLE_RATE, 15)
        self._device.write8(CONFIG, 0x00)
        self._device.write8(GYRO_CONFIG, 0x18)
        self._device.write8(ACC_CONFIG, 0x10)
        self._device.write8(INT_ENABLE, 0x01)
        self._device.write8(PWR_MGMT_1, 0x01)

    def read_Sensor(self, sensor):  
        high = self._device.readU8(sensor)
        low = self._device.readU8(sensor+1)
        data = (high << 8) | low
        if(data > 32768):
                data = data - 65536
        return data

    def get_Gyro(self):
        gyro_x = self.read_Sensor(GYRO_XOUT_H)
        gyro_y = self.read_Sensor(GYRO_YOUT_H)
        gyro_z = self.read_Sensor(GYRO_ZOUT_H)
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        return Gx, Gy, Gz    
    
    def get_Temp(self):
        temp_out = self.read_Sensor(TEMP_OUT)
        Temp = (temp_out / 340.0) + 36.53
        return Temp
    
    def get_Acc(self):
        acc_x = self.read_Sensor(ACC_XOUT_H)
        acc_y = self.read_Sensor(ACC_YOUT_H)
        acc_z = self.read_Sensor(ACC_ZOUT_H)
        Ax = acc_x/4096.0
        Ay = acc_y/4096.0
        Az = acc_z/4096.0
        return Ax, Ay, Az 
    
# Ex:   a,b,c = mpu.get_Acc() to get 3 values
    def get_RP_Angle(self):
        Ax, Ay, Az = self.get_Acc()
        pitch = -math.atan2(Ax, math.sqrt(pow(Ay,2)+pow(Az,2)))*180/(math.pi)
        roll = math.atan2(Ay, math.sqrt(pow(Ax,2)+pow(Az,2)))*180/(math.pi)
        return roll, pitch
