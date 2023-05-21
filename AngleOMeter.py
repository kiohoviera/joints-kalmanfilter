from Kalman import KalmanAngle
import smbus
import time
import math

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = True
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0

# address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47


def MPU_Init():
    bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)
    bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)
    bus.write_byte_data(DeviceAddress, CONFIG, int('0000110', 2))
    bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)
    bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
    high = bus.read_byte_data(DeviceAddress, addr)
    low = bus.read_byte_data(DeviceAddress, addr + 1)

    value = ((high << 8) | low)

    if value > 32768:
        value = value - 65536
    return value


bus = smbus.SMBus(1)
DeviceAddress = 0x68

MPU_Init()

time.sleep(1)
# Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

if RestrictPitch:
    roll = math.atan2(accY, accZ) * radToDeg
    pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
else:
    roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
    pitch = math.atan2(-accX, accZ) * radToDeg
print(roll)
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
        accX = read_raw_data(ACCEL_XOUT_H)
        accY = read_raw_data(ACCEL_YOUT_H)
        accZ = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyroX = read_raw_data(GYRO_XOUT_H)
        gyroY = read_raw_data(GYRO_YOUT_H)
        gyroZ = read_raw_data(GYRO_ZOUT_H)

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

            if (abs(kalAngleY) > 90):
                gyroXRate = -gyroXRate
                kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

        # angle = (rate of change of angle) * change in time
        gyroXAngle = gyroXRate * dt
        gyroYAngle = gyroYAngle * dt

        compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
        compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

        if (gyroXAngle < -180) or (gyroXAngle > 180):
            gyroXAngle = kalAngleX
        if (gyroYAngle < -180) or (gyroYAngle > 180):
            gyroYAngle = kalAngleY

        print("Angle X: " + str(kalAngleX) + "   " + "Angle Y: " + str(kalAngleY))
        time.sleep(0.005)

    except Exception as exc:
        flag += 1
