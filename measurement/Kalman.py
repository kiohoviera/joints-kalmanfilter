#Kalman Filter MPU6050
class KalmanAngle:
    def __init__(self):
        self.QAngle = 0.001
        self.QBias = 0.003
        self.RMeasure = 0.03
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P=[[0.0,0.0],[0.0,0.0]]

    def getAngle(self,newAngle, newRate,dt):
        self.rate = newRate - self.bias;
        self.angle += dt * self.rate;

        self.P[0][0] += dt * (dt*self.P[1][1] -self.P[0][1] - self.P[1][0] + self.QAngle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.QBias * dt

        y = newAngle - self.angle

        s = self.P[0][0] + self.RMeasure

        K=[0.0,0.0]
        K[0] = self.P[0][0]/s
        K[1] = self.P[1][0]/s

        self.angle += K[0] * y
        self.bias  += K[1] * y

        P00Temp = self.P[0][0]
        P01Temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00Temp
        self.P[0][1] -= K[0] * P01Temp
        self.P[1][0] -= K[1] * P00Temp
        self.P[1][1] -= K[1] * P01Temp

        return self.angle

    def setAngle(self,angle):
        self.angle = angle

    def setQAngle(self,QAngle):
        self.QAngle = QAngle

    def setQBias(self,QBias):
        self.QBias = QBias

    def setRMeasure(self,RMeasure):
        self.RMeasure = RMeasure

    def getRate(self):
        return self.rate

    def getQAngle(self):
        return self.QAngle

    def getQBias(self):
        return self.QBias

    def  getRMeasure(self):
        return self.RMeasure
