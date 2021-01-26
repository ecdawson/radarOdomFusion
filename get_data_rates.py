import numpy as np
import matplotlib.pyplot as plt

imu = np.genfromtxt('imu.csv', delimiter=',')
speed = np.genfromtxt('vehicle_speed.csv', delimiter=',')
imu[:, 2] = imu[:, 2]/ 10**6
speed[:, 2] = speed[:, 2]/ 10**6

diff_imu = []
diff_speed = []

for i in range(1, len(imu[:,2])-1):
    diff = imu[i, 2] - imu[i-1, 2]
    diff_imu.append(diff)

for i in range(1, len(speed[:, 2])-1):
    diff = speed[i, 2] - speed[i-1, 2]
    diff_speed.append(diff)

plt.figure(1)
plt.plot(diff_imu)

plt.figure(2)
plt.plot(diff_speed)

plt.show()

