import rcpy.mpu9250 as mpu9250
import rcpy.motor as motor
import numpy as np

imu = mpu9250.IMU(enable_dmp = True, dmp_sample_rate = 100, enable_magnetometer = True)

motors = np.array([motor.Motor(i) for i in range(1,4)])
