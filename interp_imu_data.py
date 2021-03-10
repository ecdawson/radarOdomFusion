import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

radar_data = np.genfromtxt('r1r2estimates.csv', delimiter=',')
speed = np.genfromtxt('vehicle_speed.csv', delimiter=',')
imu = np.genfromtxt('imu.csv', delimiter=',')


imu[:, 2] = imu[:, 2] / 1000000  # convert timestamp to ms
speed[:, 2] = speed[:, 2] / 1000000  # convert timestamp to ms


times = []
interp_fx = []
interp_fy = []
interp_fz = []
interp_wz = []

# align starting times
i_imu = 1
i_rad = 1
init_time_diff = imu[i_imu, 2] - radar_data[i_rad, 0]

# imu[:, 2] = signal.savgol_filter(imu[:, 2], 39, 3)
# imu[:, 3] = signal.savgol_filter(imu[:, 3], 39, 3)
# imu[:, 4] = signal.savgol_filter(imu[:, 4], 39, 3)
# imu[:, 5] = signal.savgol_filter(imu[:, 5], 39, 3)

if init_time_diff < -5:
    while init_time_diff < -5:
        # radar is ahead of imu
        i_imu = i_imu + 1
        init_time_diff = imu[i_imu, 2] - radar_data[i_rad, 0]

if init_time_diff > 5:
    while init_time_diff > 5:
        # imu is ahead of rad
        i_rad = i_rad + 1
        init_time_diff = imu[i_imu, 2] - radar_data[i_rad, 0]

cur_time = radar_data[i_rad, 0]
interp_fx.append((imu[i_imu, 29]))
interp_fy.append((imu[i_imu, 30]))
interp_fz.append((imu[i_imu, 31]))
interp_wz.append((imu[i_imu, 19]))

times.append(cur_time)

# times are now alligned
k = 0
count = 0
synched_count = 0
rad_ahead_count = 0
while k < 2400:

    current_imu = imu[i_imu, :]
    i_rad = i_rad + 1
    i_imu = i_imu + 1

    rad_next = radar_data[i_rad, 2]
    imu_next = imu[i_imu, 2]
    next_time_diff = imu[i_imu, 2] - radar_data[i_rad, 0]
    if next_time_diff < -5:
        # rad is ahead of the imu
        # this should only remain true for one cycle, because radar samples at higher rate
        next_imu = imu[i_imu, :]
        while next_time_diff < -5:
            # save imu sample normally
            # print('rad ahead')
            cur_time = imu[i_imu, 2]
            interp_fx.append((imu[i_imu, 29]))
            interp_fy.append((imu[i_imu, 30]))
            interp_fz.append((imu[i_imu, 31]))
            interp_wz.append((imu[i_imu, 19]))
            times.append(cur_time)
            rad_ahead_count = rad_ahead_count + 1
            i_imu = i_imu + 1
            next_time_diff = imu[i_imu, 2] - radar_data[i_rad, 0]

    if next_time_diff > 5:
        # imu is ahead of rad
        # if this is true need to interpolate new imu values to line up with the radar measurements
        # fill imu slots with nans where there is a rad sample but no imu sample
        while next_time_diff > 5:
            # print('imu ahead')
            cur_time = radar_data[i_rad, 0]
            interp_fx.append(np.nan)
            interp_fy.append(np.nan)
            interp_fz.append(np.nan)
            interp_wz.append(np.nan)
            count = count+1
            times.append(cur_time)
            i_rad = i_rad + 1
            next_time_diff = imu[i_imu, 2] - radar_data[i_rad, 0]

# when loop breaks out, append normally because this means the times are aligned

    synched_count = synched_count + 1
    cur_time = radar_data[i_rad, 0]
    interp_fx.append((imu[i_imu, 29]))
    interp_fy.append((imu[i_imu, 30]))
    interp_fz.append((imu[i_imu, 31]))
    interp_wz.append((imu[i_imu, 19]))
    # print('synched')
    times.append(cur_time)

    k = k + 1

# interpolate data
s_wz = pd.Series(interp_wz)
s_fx = pd.Series(interp_fx)
s_fy = pd.Series(interp_fy)
s_fz = pd.Series(interp_fz)

# interp_wz = s_wz.interpolate(method='linear')
# interp_fx = s_fx.interpolate(method='linear')
# interp_fy = s_fy.interpolate(method='linear')
# interp_fz = s_fz.interpolate(method='linear')

interp_wz = s_wz.interpolate(method='polynomial', order=2)
interp_fx = s_fx.interpolate(method='polynomial', order=2)
interp_fy = s_fy.interpolate(method='polynomial', order=2)
interp_fz = s_fz.interpolate(method='polynomial', order=2)

interp_wz = signal.savgol_filter(interp_wz, 39, 3)
interp_fx = signal.savgol_filter(interp_fx, 39, 3)
interp_fy = signal.savgol_filter(interp_fy, 39, 3)
interp_fz = signal.savgol_filter(interp_fz, 39, 3)

imu_interpolated = np.column_stack((times, interp_wz, interp_fx, interp_fy, interp_fz))

print(np.shape(imu_interpolated))
np.savetxt('imu_interpolated.csv', (times, interp_wz, interp_fx, interp_fy, interp_fz), delimiter=',')

# np.savetxt('imu_interpolated.csv', (times, f_x, f_y, f_z, w_z, forward_ave_vel, forward_vel), delimiter=',')

