import time
from AngleMeterLeft import AngleMeterLeft

baud_left = 0
angleMeterLeft = AngleMeterLeft()
angleMeterLeft.measure()

while True:
    print(angleMeterLeft.get_kalman_roll(),",", angleMeterLeft.get_complementary_roll(), ",",angleMeterLeft.get_kalman_pitch(),",", angleMeterLeft.get_complementary_pitch(),".")
    time.sleep(0.3)

