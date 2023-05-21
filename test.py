from mpu6050 import mpu6050
from Kalman import KalmanAngle
import math
import time

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()
RestrictPitch = True
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0

sensor = mpu6050(0x68)

accel_data = sensor.get_accel_data()
gyro_data = sensor.get_gyro_data()

accX = accel_data['x']
accY = accel_data['y']
accZ = accel_data['z']

if RestrictPitch:
    roll = math.atan2(accY, accZ) * radToDeg
    pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
else:
    roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
    pitch = math.atan2(-accX, accZ) * radToDeg
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll
gyroYAngle = pitch
compAngleX = roll
compAngleY = pitch

timer = time.time()
flag = 0
while True:
    if flag > 100:
        print("There is a problem with the connection")
        flag = 0
        continue
    try:
        # Read Accelerometer raw value
        accX = accel_data['x']
        accY = accel_data['y']
        accZ = accel_data['z']

        # Read Gyroscope raw value
        gyroX = gyro_data['x']
        gyroY = gyro_data['y']
        gyroZ = gyro_data['z']

        dt = time.time() - timer
        timer = time.time()

        if RestrictPitch:
            roll = math.atan2(accY, accZ) * radToDeg
            pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
        else:
            roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
            pitch = math.atan2(-accX, accZ) * radToDeg

        gyroXRate = gyroX / 131
        gyroYRate = gyroY / 131

        if RestrictPitch:

            if (roll < -90 and kalAngleX > 90) or (roll > 90 and kalAngleX < -90):
                kalmanX.setAngle(roll)
                complAngleX = roll
                kalAngleX = roll
                gyroXAngle = roll
            else:
                kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

            if abs(kalAngleX) > 90:
                gyroYRate = -gyroYRate
                kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)
        else:

            if (pitch < -90 and kalAngleY > 90) or (pitch > 90 and kalAngleY < -90):
                kalmanY.setAngle(pitch)
                complAngleY = pitch
                kalAngleY = pitch
                gyroYAngle = pitch
            else:
                kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)

            if abs(kalAngleY) > 90:
                gyroXRate = -gyroXRate
                kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

        gyroXAngle = gyroXRate * dt
        gyroYAngle = gyroYAngle * dt

        compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
        compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

        if ((gyroXAngle < -180) or (gyroXAngle > 180)):
            gyroXAngle = kalAngleX
        if ((gyroYAngle < -180) or (gyroYAngle > 180)):
            gyroYAngle = kalAngleY

        print("Angle X: " + str(kalAngleX) + "   " + "Angle Y: " + str(kalAngleY))
        time.sleep(0.005)

    except Exception as exc:
        flag += 1
