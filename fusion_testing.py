from data_synchronization import data_synch
import estimations
from riss import riss
import numpy as np
from scipy import io
import matplotlib.pyplot as plt
# from adapted_kf_meter import kf_meter
# from change_disp_kf import ChangeDispKF
from change_disp_kf_w import ChangeDispKF

floor_map = io.loadmat('floor_map.mat')
imu_odo = np.genfromtxt('imu_and_odo.csv', delimiter=',')
imu_odo = np.transpose(imu_odo)

# 'time', 'velocity_r1',' angular_vel_r1', 'num_detections1', 'num_inliers1', 'average_range1',
#          'velocity_r2', 'angular_vel_r2', 'num_detections2', 'num_inliers2', 'average_range2', 'combined_velocity',
#          'combined_angular_vel', 'x', 'y'
radar_data = np.genfromtxt('r1r2estimates.csv', delimiter=',')

# rad dat time, x, y, v, w
rad_dat = np.column_stack((radar_data[1:-1, 0], radar_data[1:-1, 13], radar_data[1:-1, 14], radar_data[1:-1, 11],
                           radar_data[1:-1, 12]))

synch_for_fusion = data_synch(imu_odo, rad_dat)
# initialize indices
i_rad = 0
i_imu = 0
gyro_bias = 0.0
time_stamps = []

rad_sample, imu_sample, i_rad, i_riss, time = synch_for_fusion.get_synched_samples(i_rad, i_imu)

time_stamps.append(time)

gaps_array = []
time_differences = []
x_pos_errors = []
y_pos_errors = []
P_set1 = []
P_set2 = []
P_set3 = []
P_set4 = []
P_set5 = []
P_set6 = []
P_set7 = []
P_set8 = []
P_set9 = []
corrected_x = []
corrected_y = []
radar_x = []
radar_y = []
previous_time = time

# instantiate kf meter class
initial_position_estimate = [46.57, 30.91, 0, 0, 0, 0, 0, 0, 0]
# kf = kf_meter(0, 0, 1, initial_position_estimate, 0, 0)
disp_kf = ChangeDispKF(0, 0, 1, initial_position_estimate, 0, 0, 0, 0)

# initialize riss class
x_m = 46.57
y_m = 30.91
azi = np.rad2deg(-3.13629907)

single_metre = riss(0, 0, 0, azi, x_m, y_m)
untouched_riss = riss(0, 0, 0, azi, x_m, y_m)

i_rad = i_rad + 1
i_riss = i_riss + 1
x_rad = x_m
y_rad = y_m
azi_rad = -3.13629907
omega = 0
acceleration = 0

incremental_x = 46.57
incremental_y = 30.91
incremental_a = np.rad2deg(-3.13629907)

incremental_rad_x = 46.57
incremental_rad_y = 30.91
incremental_rad_a = -3.13629907

inc_x = []
inc_y = []
inc_a = []
inc_rad_x = []
inc_rad_y = []
inc_rad_a = []

change_in_x_riss = []
change_in_y_riss = []
change_in_a_riss = []

change_in_x_rad = []
change_in_y_rad = []
change_in_a_rad = []

untouched_x = x_m
untouched_y = y_m

unt_x = []
unt_y = []

z_vectors = []

azi_radar_change = []
azi_riss = []

for i in range(1, 2000):
    rad_sample, imu_sample, i_rad, i_imu, time = synch_for_fusion.get_synched_samples(i_rad, i_imu)

    # run riss here, get "riss sample" which will then be used to calculate the Z matrix
    fx = imu_sample[1]
    fy = imu_sample[2]
    fz = imu_sample[3]
    wz = imu_sample[4]
    velocity = imu_sample[5]

    next_time = time
    i_rad = i_rad + 1
    i_riss = i_riss + 1
    time_stamps.append(time)
    gap, time_diff = synch_for_fusion.is_gap(previous_time, next_time)
    time_diff = time_diff / 1000

    # RISS UPDATE
    # x_pos, y_pos, vx, vy, h, azi, accel, pitch, roll, del_x, del_y, del_a = single_metre.update_metres(fx, fy,
    # wz, time_diff, gyro_bias,velocity)

    vx, vy, h, accel, pitch, roll, del_x, del_y, del_a = single_metre.update_disp_meters(fx, fy, wz, time_diff,
                                                                                         gyro_bias, velocity)
    un_vx, un_vy, _, _, _, _, un_x, un_y, un_a = untouched_riss.update_disp_meters(fx, fy, wz, time_diff,
                                                                                        gyro_bias, velocity)
    untouched_x = untouched_x + un_x
    untouched_y = untouched_y + un_y
    unt_x.append(untouched_x)
    unt_y.append(untouched_y)

    # integration for updated position
    incremental_x = incremental_x + del_x
    incremental_y = incremental_y + del_y
    incremental_a = incremental_a + del_a

    change_in_x_riss.append(del_x)
    change_in_y_riss.append(del_y)
    change_in_a_riss.append(del_a)

    inc_x.append(incremental_x)
    inc_y.append(incremental_y)
    inc_a.append(incremental_a)

    # RADAR UPDATE
    rad_vel = rad_sample[3]
    rad_w = rad_sample[4] + 0.15
    x_rad, y_rad, azi_rad, del_rad_x, del_rad_y, del_rad_a, vx_rad, vy_rad = estimations.update_radar_pos(x_rad, y_rad,
                                                                                azi_rad, rad_vel, rad_w, time_diff)
    azi_radar_change.append(del_rad_a)
    azi_riss.append(del_a)

    incremental_rad_x = incremental_rad_x + del_rad_x
    incremental_rad_y = incremental_rad_y + del_rad_y
    incremental_rad_a = incremental_rad_a + del_rad_a

    change_in_x_rad.append(del_rad_x)
    change_in_y_rad.append(del_rad_y)
    change_in_a_rad.append(del_rad_a)

    inc_rad_x.append(incremental_rad_x)
    inc_rad_y.append(incremental_rad_y)
    inc_rad_a.append(incremental_rad_a)
    print(rad_w, wz)
    # compute Z matrix: subtract x,y,vx,vy,Azimuth
    # Z = [x_pos - rad_sample[1], y_pos - rad_sample[2], vx - rad_sample[3],
    # vy - rad_sample[4], azi - rad_sample[5]]

    # z_diff = [del_x - del_rad_x, del_y - del_rad_y, vx - vx_rad, vy - vy_rad, del_a - np.rad2deg(del_rad_a)]
    z_diff = [del_x - del_rad_x, del_y - del_rad_y, vx - vx_rad, vy - vy_rad, wz - rad_w, del_a - np.rad2deg(del_rad_a)]
    z_vectors.append(z_diff)
    # Z = [x_pos-x_rad, y_pos-y_rad,vx-rad_sample[3],
    # vy-rad_sample[4],azi-azi_rad]

    # option only updating vx and vy, as these are 'measurements' independent of previous state
    # Z = [vx-rad_sample[3],vy-rad_sample[4]]

    # send Z matrix into KF with time_diff, azi and acceleration from odometer
    # direction_flag = data_rad1[i_rad - 1, 7]
    # num_targets = data_rad1[i_rad - 1, 5]
    # num_inliers = data_rad1[i_rad - 1, 6]

    disp_kf.tune_R(None, None, None)
    # corrections, P = kf.update(Z, azi, accel, time_diff, pitch)

    corrections, P = disp_kf.update(z_diff, del_a, accel, time_diff, pitch, velocity, wz)

    x_pos_errors.append(corrections[0])
    y_pos_errors.append(corrections[1])

    P_set1.append(P[0, 0])
    P_set2.append(P[1, 1])
    P_set3.append(P[2, 2])
    P_set4.append(P[3, 3])
    P_set5.append(P[4, 4])
    P_set6.append(P[5, 5])
    P_set7.append(P[6, 6])
    P_set8.append(P[7, 7])
    P_set9.append(P[8, 8])

    x_pos = incremental_x - corrections[0]
    y_pos = incremental_y - corrections[1]
    # h is position 2
    # vx = vx - corrections[3]
    # vy = vy - corrections[4]
    # vu is position 5
    azi = incremental_a - corrections[6]
    acceleration = acceleration - corrections[7]
    omega = omega - corrections[8]

    radar_x.append(x_rad)
    radar_y.append(y_rad)

    # x_rad = x_pos
    # y_rad = y_pos
    # azi_rad = azi

    corrected_x.append(x_pos)
    corrected_y.append(y_pos)

    # correct variables
    single_metre.correct_values(x_pos, y_pos, azi)

    # kf returns
    gaps_array.append(gap)
    time_differences.append(time_diff)
    previous_time = next_time
    # print(corrections)

z_vectors = np.array(z_vectors)
# kalman filter, riss and radar plots

# plt.figure(20)
# plt.plot(azi_riss, label='riss azi del')
# plt.plot(azi_radar_change, label='radar')
# plt.legend()
# plt.show()

plt.figure(1)
plt.plot(corrected_x, corrected_y, label='KF')
# plt.plot(riss_results[:, 1], riss_results[:, 2], label='riss')
# plt.plot(radar[:,1],radar[:,2],label='radar')
plt.plot(unt_x, unt_y, label='riss alone')
plt.plot(inc_rad_x, inc_rad_y, label='radar alone')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.legend()
plt.ylabel('Y Position (m)')
plt.xlabel('X Position (m)')


# # P matrix
plt.figure(2)
plt.plot(P_set1, label='x')
plt.plot(P_set2, label='y')
# plt.plot(P_set3,label='vx')
plt.plot(P_set4, label='vy')
# plt.plot(P_set5,label='h')
plt.plot(P_set6, label='A')
# plt.plot(P_set7,label='wz')
# plt.plot(P_set8,label='a')
# plt.plot(P_set9,label='bias')
plt.legend()

plt.figure(13)
plt.plot(z_vectors[:, 0], label='change in x')
plt.legend()
plt.figure(14)
plt.plot(z_vectors[:, 1], label='change in y')
plt.legend()
plt.figure(15)
plt.plot(z_vectors[:, 2], label='vx')
plt.legend()
plt.figure(16)
plt.plot(z_vectors[:, 3], label='vy')
plt.legend()
plt.figure(17)
plt.plot(z_vectors[:, 4], label='change in azimuth')
plt.legend()
plt.show()
#
# # inliers ratio. inliers/total number of targets in scan
# plt.figure(3)
# plt.plot(data_rad1[:, 0], inliers_ratio)
# plt.ylabel('Y Position (m)')
# plt.xlabel('Time (ms)')
#
# # reversing flag, 1 if forward, 0 if rev
# plt.figure(4)
# plt.plot(data_rad1[:, 0], data_rad1[:, 7])
# plt.ylabel('Reverse flag 0 if reverse, 1 if forward')
# plt.xlabel('Time (ms)')
#
# # number of points in scan
# plt.figure(5)
# plt.plot(data_rad1[:, 0], data_rad1[:, 5])
# plt.ylabel('Number of points in scan')
# plt.xlabel('Time (ms)')
#
# # time difference between kf updates
# # plt.figure(6)
# # plt.plot(time_stamps[:1399], time_differences)
# # plt.ylabel('Time difference between position updates')
# # plt.xlabel('Time (ms)')
#
# # plt.figure(7)
# # plt.plot(change_in_x_riss, label = 'riss')
# # plt.plot(change_in_x_rad, label = 'rad')
# # plt.ylabel('Incremental change in X Position')
# # plt.legend()
# #
# #
# # plt.figure(8)
# # plt.plot(change_in_y_riss, label = 'riss')
# # plt.plot(change_in_y_rad, label = 'rad')
# # plt.ylabel('Incremental change in Y Position')
# # plt.legend()
# #
# # plt.figure(9)
# # plt.plot(change_in_a_riss,label = 'riss')
# # plt.plot(np.rad2deg(change_in_a_rad), label = 'rad')
# # plt.ylabel('Incremental change in Azimuth (degrees)')
# # plt.legend()
#
# plt.figure(7)
# plt.plot(np.array(change_in_x_riss)-np.array(change_in_x_rad), label = 'diff')
# plt.ylabel('Difference in Riss and Rad Incremental change in X Position')
# plt.legend()
#
#
# plt.figure(8)
# plt.plot(np.array(change_in_y_riss)-np.array(change_in_y_rad), label = 'diff')
# plt.ylabel('Difference in Riss and Rad incremental change in Y Position')
# plt.legend()
#
# plt.figure(9)
# plt.plot(np.array(change_in_a_riss)-np.array(np.rad2deg(change_in_a_rad)),label = 'diff')
# plt.ylabel('Difference in Riss and Rad Azimuth (degrees)')
# plt.legend()
#
# plt.figure(10)
# plt.plot(x_pos_errors,label = 'x pos errors from kf')
# plt.plot(y_pos_errors,label = 'y pos errors from kf')
# plt.ylabel('Error in Change in Position from KF')
# plt.legend()
#
# plt.figure(11)
# plt.plot(radar[:,1],radar[:,2], label = 'radar left')
# plt.plot(radar2[:,1],radar2[:,2], label = 'radar right')
# plt.plot(inc_x, inc_y, label = 'riss')
# plt.legend()
#
#
#
# # fwdvelocities
# plt.figure(12)
# plt.plot(data_rad1[:,0], data_rad1[:,1],label = 'rad1')
# plt.plot(data_rad2[:,0], data_rad2[:,1],label = 'rad2')
# plt.legend()
# plt.ylabel('fwd vels')
# #angular velocities
#
# plt.figure(13)
# plt.plot(data_rad1[:,0], data_rad1[:,2],label = 'rad1')
# plt.plot(data_rad2[:,0], data_rad2[:,2],label = 'rad2')
# plt.ylabel('angular vels')
# plt.legend()
# plt.show()
