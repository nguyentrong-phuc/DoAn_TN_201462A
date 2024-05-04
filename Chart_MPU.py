import matplotlib
matplotlib.use('Agg')  
import matplotlib.pyplot as plt

mpu = Adafruit_PCA9685.MPU6050()

num_samples = 100
sample_interval = 0.1
timestamps = []
roll = []
pitch = []

def read_sensor_data():
    global roll, pitch
    roll_angle, pitch_angle = mpu.get_RP_Angle()
    roll.append(roll_angle)
    pitch.append(pitch_angle)
    timestamps.append(time.time())

for _ in range(num_samples):
    read_sensor_data()
    time.sleep(sample_interval)

plt.figure(figsize=(10, 6))
plt.plot(timestamps, roll, label='Roll')
plt.plot(timestamps, pitch, label='Pitch')
plt.xlabel('Time')
plt.ylabel('Angle (degrees)')
plt.title('MPU6050 Roll and Pitch over Time')
plt.legend()
plt.grid(True)
plt.savefig('mpu6050_plot.png')
