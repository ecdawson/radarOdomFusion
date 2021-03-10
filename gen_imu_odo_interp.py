import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal


# this file generates a synchronized imu with odometer file, and also finds the times when the vehicle is reversing
# based on what the radar indicates, and makes odometer readings negative during these times
speed = np.genfromtxt('vehicle_speed.csv', delimiter=',')
imu = np.genfromtxt('imu_interpolated.csv', delimiter=',')
data_rad1 = np.genfromtxt('vel_estimation_R1.csv', delimiter=',')
imu = np.transpose(imu)
print(np.shape(imu))
# imu file: times, interp_wz, interp_fx, interp_fy, interp_fz
imu[:, 1] = -imu[:, 1]


# imu = imu[:3100, :]
speed[:, 2] = speed[:, 2] / 1000000  # convert timestamp to ms

# use wheel tex data to know when vehicle is going backwards
# do this manually in sections 300 to 3000, 3000 to 4000, 4000 to 5000

# ONCE
data_rad1check = data_rad1[:, 300:3000]
reversing_indices = np.where(data_rad1check[1, :] < 0)
reversing_times = np.array(data_rad1check[0, reversing_indices])

array = np.asarray(speed[1:-1, 2])
first_rev = reversing_times[0, 0]
last_rev = reversing_times[0, -1]
idx_first = (np.abs(array - first_rev)).argmin()
idx_last = (np.abs(array - last_rev)).argmin()

reversing_times = np.array(reversing_times)
speed[idx_first:idx_last, 4] = -speed[idx_first:idx_last, 4]

# TWICE
data_rad1check = data_rad1[:, 3000:4000]
reversing_indices = np.where(data_rad1check[1, :] < 0)
reversing_times = np.array(data_rad1check[0, reversing_indices])

array = np.asarray(speed[1:-1, 2])
first_rev = reversing_times[0, 0]
last_rev = reversing_times[0, -1]
idx_first = (np.abs(array - first_rev)).argmin()
idx_last = (np.abs(array - last_rev)).argmin()

reversing_times = np.array(reversing_times)
speed[idx_first:idx_last, 4] = -speed[idx_first:idx_last, 4]

# THREE TIMES
data_rad1check = data_rad1[:, 4000:5500]
reversing_indices = np.where(data_rad1check[1, :] < 0)
reversing_times = np.array(data_rad1check[0, reversing_indices])

array = np.asarray(speed[1:-1, 2])
first_rev = reversing_times[0, 0]
last_rev = reversing_times[0, -1]
idx_first = (np.abs(array - first_rev)).argmin()
idx_last = (np.abs(array - last_rev)).argmin()

reversing_times = np.array(reversing_times)
speed[idx_first:idx_last, 4] = -speed[idx_first:idx_last, 4]

speed_rates = []
for i in range(1, len(speed[:, 0])-1):
    speed_diff = speed[i+1, 2] - speed[i, 2]
    speed_rates.append(speed_diff)

imu_rates = []
for i in range(1, len(imu[:, 0]) - 1):
    imu_diff = imu[i + 1, 2] - imu[i, 2]
    imu_rates.append(imu_diff)

# variables for returning
time = []
f_x = []
f_y = []
f_z = []
w_z = []
forward_vel = []
forward_ave_vel = []

# align starting times
i_imu = 1
i_speed = 1
init_time_diff = imu[i_imu, 0] - speed[i_speed, 2]

if init_time_diff < -5:
    while init_time_diff < -5:
        # speed is ahead of imu
        i_imu = i_imu + 1
        init_time_diff = imu[i_imu, 2] - speed[i_speed, 2]

if init_time_diff > 5:
    while init_time_diff > 5:
        # imu is ahead of speed. speed samples at 100 hz so want to save these values
        i_speed = i_speed + 1
        init_time_diff = imu[i_imu, 0] - speed[i_speed, 2]
k = 0
while k < 4500:
    # get next time
    i_speed = i_speed + 1
    i_imu = i_imu + 1
    speed_next = speed[i_speed, 2]
    imu_next = imu[i_imu, 0]
    next_time_diff = imu[i_imu, 0] - speed[i_speed, 2]

    # imu samples every 150 to 200 ms. odometer samples every 10ms.
    # imu time = 51ms, odometer time = 53ms. these have to match
    # there will be a match if the time diff is less than +/-5.
    sum_speed = 0
    sum_counter = 0
    # this loop finds one odo measurement and one set of inertial measurements to use for RISS
    if next_time_diff < -5:
        while next_time_diff < -5:
            # speed is ahead of imu
            i_imu = i_imu + 1
            next_time_diff = imu[i_imu, 0] - speed[i_speed, 2]
            print('speed ahead')
            print(next_time_diff)

    if next_time_diff > 5:
        while next_time_diff > 5:
            # imu is ahead of speed. speed samples at 100 hz so want to save these values to downsample by averaging
            sum_speed = sum_speed + speed[i_speed, 4]
            sum_counter = sum_counter + 1
            i_speed = i_speed + 1
            next_time_diff = imu[i_imu, 0] - speed[i_speed, 2]

    if sum_counter is not 0:
        ave_vel = sum_speed/sum_counter
    else:
        ave_vel = speed[i_speed, 4]
    # get parameters for RISS
    cur_time = speed[i_speed, 2]
    fx = imu[i_imu, 2]
    fy = imu[i_imu, 3]
    fz = imu[i_imu, 4]
    wz = imu[i_imu, 1]
    velocity = speed[i_speed, 4]

    time.append(cur_time)
    f_x.append(fx)
    f_y.append(fy)
    f_z.append(fz)
    w_z.append(wz)
    forward_vel.append(velocity)
    forward_ave_vel.append(ave_vel)
    k = k + 1
np.savetxt('imu_and_odo_interp_ave.csv', (time, f_x, f_y, f_z, w_z, forward_ave_vel), delimiter=',')

plt.figure(1)
plt.plot(forward_ave_vel)
plt.figure(2)
plt.plot(w_z)

plt.figure(3)
plt.plot(f_x)

plt.figure(4)
plt.plot(f_y)

plt.figure(5)
plt.plot(f_z)
plt.show()
# print(np.shape(time))
# print(np.shape(imu))
# np.savetxt('speed_rev_test.csv', (time, f_x, f_y, f_z, w_z, forward_vel), delimiter=',')
