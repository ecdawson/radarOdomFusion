import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# this file generates a synchronized imu with odometer file, and also finds the times when the vehicle is reversing
# based on what the radar indicates, and makes odometer readings negative during these times
speed = np.genfromtxt('vehicle_speed.csv', delimiter=',')
imu = np.genfromtxt('imu.csv', delimiter=',')
data_rad1 = np.genfromtxt('vel_estimation_R1.csv', delimiter=',')

imu[:, 19] = -imu[:, 19]

# imu = imu[:3100, :]
imu[:, 2] = imu[:, 2] / 1000000  # convert timestamp to ms
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


# variables for returning
time = []
f_x = []
f_y = []
f_z = []
w_z = []
forward_vel = []

# align starting times
i_imu = 1
i_speed = 1
init_time_diff = imu[i_imu, 2] - speed[i_speed, 2]

if init_time_diff < -5:
    while init_time_diff < -5:
        i_imu = i_imu + 1
        init_time_diff = imu[i_imu, 2] - speed[i_speed, 2]

if init_time_diff > 5:
    while init_time_diff > 5:
        i_speed = i_speed + 1
        init_time_diff = imu[i_imu, 2] - speed[i_speed, 2]
k = 0
while k < 2500:
    # get next time
    i_speed = i_speed + 1
    i_imu = i_imu + 1
    speed_next = speed[i_speed, 2]
    imu_next = imu[i_imu, 2]
    next_time_diff = imu[i_imu, 2] - speed[i_speed, 2]

    # imu samples every 150 to 200 ms. odometer samples every 10ms.
    # imu time = 51ms, odometer time = 53ms. these have to match
    # there will be a match if the time diff is less than +/-5.

    # this loop finds one odo measurement and one set of inertial measurements to use for RISS
    if next_time_diff < -5:
        while next_time_diff < -5:
            i_imu = i_imu + 1
            next_time_diff = imu[i_imu, 2] - speed[i_speed, 2]

    if next_time_diff > 5:
        while next_time_diff > 5:
            i_speed = i_speed + 1
            next_time_diff = imu[i_imu, 2] - speed[i_speed, 2]

    # get parameters for RISS
    cur_time = speed[i_speed, 2]
    fx = imu[i_imu, 29]
    fy = imu[i_imu, 30]
    fz = imu[i_imu, 31]
    wz = imu[i_imu, 19]
    velocity = speed[i_speed, 4]

    time.append(cur_time)
    f_x.append(fx)
    f_y.append(fy)
    f_z.append(fz)
    w_z.append(wz)
    forward_vel.append(velocity)
    k = k + 1
np.savetxt('imu_and_odo.csv', (time,f_x,f_y,f_z,w_z,forward_vel), delimiter=',')
print(np.shape(time))
print(np.shape(imu))
np.savetxt('speed_rev_test.csv', (time, f_x, f_y, f_z, w_z, forward_vel), delimiter=',')
