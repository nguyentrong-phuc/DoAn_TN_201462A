from __future__ import division
import logging
import time
from time import sleep
import math 

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

logger = logging.getLogger(__name__)

class MPU6050(object):
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
        gyro_x = mpu.read_Sensor(GYRO_XOUT_H)
        gyro_y = mpu.read_Sensor(GYRO_YOUT_H)
        gyro_z = mpu.read_Sensor(GYRO_ZOUT_H)
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        # return gyro_x, gyro_y, gyro_z
        # print ("Gx=%.2f" %Gx, "  Gy=%.2f" %Gy, "  Gz=%.2f" %Gz) 
        return Gx, Gy, Gz    
    
    def get_Temp(self):
        temp_out = mpu.read_Sensor(TEMP_OUT)
        Temp = (temp_out / 340.0) + 36.53
        print("%.2f^C"%Temp)
        return Temp
    
    def get_Acc(self):
        acc_x = self.read_Sensor(ACC_XOUT_H)
        acc_y = self.read_Sensor(ACC_YOUT_H)
        acc_z = self.read_Sensor(ACC_ZOUT_H)
        Ax = acc_x/4096.0
        Ay = acc_y/4096.0
        Az = acc_z/4096.0
        #return acc_x, acc_y, acc_z
        # print ("Ax=%.2f g" %Ax, "  Ay=%.2f g" %Ay, "  Az=%.2f g" %Az) 
        return Ax, Ay, Az 
    
# Ex:   a,b,c = mpu.get_Acc() to get 3 values
        
    def get_RP_Angle(self):
        Ax, Ay, Az = self.get_Acc()
        pitch = -math.atan2(Ax, math.sqrt(pow(Ay,2)+pow(Az,2)))*180/(math.pi)
        roll = math.atan2(Ay, math.sqrt(pow(Ax,2)+pow(Az,2)))*180/(math.pi)
        print("Goc truc X: %.1f" %roll  ,  "Goc truc Y: %.1f" %pitch)
        return roll, pitch
        #return Ax, Ay, Az


mpu = MPU6050()
while True:        
    # m,n,p = mpu.get_Gyro()
    # a,b,c = mpu.get_RP_Angle()
    # t = mpu.get_Temp()
    # roll, pitch = mpu.get_RP_Angle()
    mpu.get_RP_Angle()
    sleep(0.5)
    
        
