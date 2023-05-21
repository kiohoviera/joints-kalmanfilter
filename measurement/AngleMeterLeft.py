from Kalman import KalmanAngle
import smbus2
import time
import math
import threading


class AngleMeterLeft:
    def MPU_Init(self):

        PWR_MGMT_1 = 0x6B
        SMPLRT_DIV = 0x19
        CONFIG = 0x1A
        GYRO_CONFIG = 0x1B
        INT_ENABLE = 0x38

        self.bus.write_byte_data(self.DeviceAddress, SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.DeviceAddress, PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.DeviceAddress, CONFIG, int('0000110', 2))
        self.bus.write_byte_data(self.DeviceAddress, GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.DeviceAddress, INT_ENABLE, 1)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.DeviceAddress, addr)
        low = self.bus.read_byte_data(self.DeviceAddress, addr + 1)

        value = ((high << 8) | low)

        if (value > 32768):
            value = value - 65536
        return value

    bus = smbus2.SMBus(1)
    DeviceAddress = 0x68

    def measureAngles(self):
        flag = 0
        kalmanX = KalmanAngle()
        kalmanY = KalmanAngle()

        RestrictPitch = False
        radToDeg = 57.2957786
        kalAngleX = 0
        kalAngleY = 0
        # some MPU6050 Registers and their Address

        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H = 0x43
        GYRO_YOUT_H = 0x45
        GYRO_ZOUT_H = 0x47

        time.sleep(1)
        # Read Accelerometer raw value
        accX = self.read_raw_data(ACCEL_XOUT_H)
        accY = self.read_raw_data(ACCEL_YOUT_H)
        accZ = self.read_raw_data(ACCEL_ZOUT_H)

        # print(accX,accY,accZ)
        # print(math.sqrt((accY**2)+(accZ**2)))
        if (RestrictPitch):
            roll = math.atan2(accY, accZ) * radToDeg
            pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
        else:
            roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
            pitch = math.atan2(-accX, accZ) * radToDeg
        # print(roll)
        kalmanX.setAngle(roll)
        kalmanY.setAngle(pitch)
        gyroXAngle = roll
        gyroYAngle = pitch
        compAngleX = roll
        compAngleY = pitch

        timer = time.time()
        flag = 0

        while True:

            if flag > 100:  # Problem with the connection
                print("There is a problem with the connection")
                flag = 0
                continue
            try:
                # Read Accelerometer raw value
                accX = self.read_raw_data(ACCEL_XOUT_H)
                accY = self.read_raw_data(ACCEL_YOUT_H)
                accZ = self.read_raw_data(ACCEL_ZOUT_H)

                # Read Gyroscope raw value
                gyroX = self.read_raw_data(GYRO_XOUT_H)
                gyroY = self.read_raw_data(GYRO_YOUT_H)
                gyroZ = self.read_raw_data(GYRO_ZOUT_H)

                dt = time.time() - timer
                timer = time.time()

                if (RestrictPitch):
                    roll = math.atan2(accY, accZ) * radToDeg
                    pitch = math.atan(-accX / math.sqrt((accY ** 2) + (accZ ** 2))) * radToDeg
                else:
                    roll = math.atan(accY / math.sqrt((accX ** 2) + (accZ ** 2))) * radToDeg
                    pitch = math.atan2(-accX, accZ) * radToDeg

                gyroXRate = gyroX / 131
                gyroYRate = gyroY / 131

                self.gyroYRate = gyroYRate

                if (RestrictPitch):

                    if ((roll < -90 and kalAngleX > 90) or (roll > 90 and kalAngleX < -90)):
                        kalmanX.setAngle(roll)
                        complAngleX = roll
                        kalAngleX = roll
                        gyroXAngle = roll
                    else:
                        kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

                    if (abs(kalAngleY) > 90 or True):
                        gyroYRate = -gyroYRate
                        kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)
                else:

                    if ((pitch < -90 and kalAngleY > 90) or (pitch > 90 and kalAngleY < -90)):
                        kalmanY.setAngle(pitch)
                        complAngleY = pitch
                        kalAngleY = pitch
                        gyroYAngle = pitch
                    else:
                        kalAngleY = kalmanY.getAngle(pitch, gyroYRate, dt)

                    if abs(kalAngleX) > 90:
                        gyroXRate = -gyroXRate
                        kalAngleX = kalmanX.getAngle(roll, gyroXRate, dt)

                gyroXAngle = gyroXRate * dt
                gyroYAngle = gyroYAngle * dt

                compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
                compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

                if (gyroXAngle < -180) or (gyroXAngle > 180):
                    gyroXAngle = kalAngleX
                if (gyroYAngle < -180) or (gyroYAngle > 180):
                    gyroYAngle = kalAngleY

                self.pitch = compAngleY
                self.roll = compAngleX

                self.kalman_pitch = kalAngleY
                self.kalman_roll = kalAngleX
                self.compl_pitch = compAngleY
                self.compl_roll = compAngleX
                time.sleep(0.005)

            except Exception as exc:
                if (flag == 100):
                    print(exc)
                flag += 1

    def __init__(self):
        self.gyroYRate = 0
        self.pitch = 0
        self.roll = 0
        self.MPU_Init()
        self.bus = smbus2.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
        self.DeviceAddress = 0x68  # MPU6050 device address
        self.compl_pitch = 0
        self.compl_roll = 0
        self.kalman_pitch = 0
        self.kalman_roll = 0

    def measure(self):
        angleThread = threading.Thread(target=self.measureAngles)
        angleThread.start()

    def getRoll(self):
        return self.roll

    def getPitch(self):
        return self.pitch

    def get_int_pitch(self):
        return int(self.pitch)

    def get_int_roll(self):
        return int(self.roll)

    def get_complementary_roll(self):
        return int(self.compl_roll)

    def get_complementary_pitch(self):
        return int(self.compl_pitch)

    def get_kalman_roll(self):
        return int(self.kalman_roll)

    def get_kalman_pitch(self):
        return int(self.kalman_pitch)

    def get_angular_velocity(self):
        return int(self.gyroYRate)