from data_synchronization import data_synch
import estimations
import math
import matplotlib.animation as animation
from riss import riss
import numpy as np
from scipy import io
import matplotlib.pyplot as plt
# from adapted_kf_meter import kf_meter
from kf_positions_w import kf_meter
import trajectory_errors
# from kf_vw import kf_meter

ref_sol = np.genfromtxt('ref_generated.csv', delimiter=',')
ref_sol = ref_sol[0:4500, :]
floor_map = io.loadmat('floor_map.mat')
imu_odo = np.genfromtxt('imu_and_odo.csv', delimiter=',')
# imu_odo = np.genfromtxt('imu_and_odo.csv', delimiter=',')
imu_odo = np.transpose(imu_odo)

# 'time', 'velocity_r1',' angular_vel_r1', 'num_detections1', 'num_inliers1', 'average_range1',
#          'velocity_r2', 'angular_vel_r2', 'num_detections2', 'num_inliers2', 'average_range2', 'combined_velocity',
#          'combined_angular_vel', 'x', 'y'
radar_data = np.genfromtxt('r1r2estimates.csv', delimiter=',')

# rad dat time, x, y, v, w
# X AND Y ARE MISTAKENLY SWITCHED
rad_dat = np.column_stack((radar_data[1:-1, 0], radar_data[1:-1, 13], radar_data[1:-1, 14], radar_data[1:-1, 11],
                           radar_data[1:-1, 12], radar_data[1:-1, 4], radar_data[1:-1, 9]))

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

reference_x = []
reference_y = []
# instantiate kf meter class
initial_position_estimate = [46.57, 30.91, 0, 0.2, 0.2, 0, 0, 0, 0]
kf = kf_meter(0, 0, 1, initial_position_estimate, 0, 0)

# initialize riss class
x_m = 46.57
y_m = 30.91
azi_m = -3.13629907

single_metre = riss(0, 0, 0, np.rad2deg(-3.13629907), x_m, y_m)
untouched_riss = riss(0, 0, 0, np.rad2deg(-3.13629907), x_m, y_m)

untouched_x = x_m
untouched_y = y_m

i_rad = i_rad + 1
i_riss = i_riss + 1
x_rad = x_m
y_rad = y_m
azi_rad = azi_m #+ math.pi / 2
omega = 0
acceleration = 0

z_vectors = []
radar_azimuths = []
radar_velocities = []
riss_azimuths = []
riss_velocities = []
unt_x = []
unt_y = []

vx_rad_array = []
vy_rad_array = []
vx_riss = []
vy_riss = []
radar_w = []
riss_w = []
odo_vel = []

corrections = np.zeros(9)
correction_data = []
corr_vx = 0
corr_vy = 0

untouched_vx = []
untouched_vy = []

outage_starts = []
outage_ends = []
outage_positions = []

out_flag = []

for i in range(1, 2000):
    rad_sample, imu_sample, i_rad, i_imu, time = synch_for_fusion.get_synched_samples(i_rad, i_imu)

    # run riss here, get "riss sample" which will then be used to calculate the Z matrix
    fx = imu_sample[1]
    fy = imu_sample[2]
    fz = imu_sample[3]
    wz = -imu_sample[4]
    w_original = -imu_sample[4]
    velocity = imu_sample[5]

    next_time = time
    i_rad = i_rad + 1
    i_riss = i_riss + 1
    time_stamps.append(time)
    gap, time_diff = synch_for_fusion.is_gap(previous_time, next_time)
    time_diff = time_diff / 1000

    # CORRECT PARAMETERS USING PREVIOUS KF CORRECTIONS
    # wz = wz - corrections[8]


    # RISS UPDATE
    x_pos, y_pos, vx, vy, h, azi, accel, pitch, roll, del_x, del_y, del_a = single_metre.update_metres(fx, fy,
        wz, time_diff, gyro_bias,velocity, corr_vx, corr_vy, 1)

    un_vx, un_vy, _, _, _, _, un_x, un_y, un_a = untouched_riss.update_disp_meters(fx, fy, w_original, time_diff,
                                                                             gyro_bias, velocity, 0)
    odo_vel.append(velocity)
    untouched_x = untouched_x + un_x
    untouched_y = untouched_y + un_y
    unt_x.append(untouched_x)
    unt_y.append(untouched_y)
    untouched_vx.append(un_vx)
    untouched_vy.append(un_vy)
    # vx, vy, h, accel, pitch, roll, del_x, del_y, del_a = single_metre.update_disp_meters(fx, fy, wz, time_diff,
            # gyro_bias, velocity)

    # RADAR UPDATE
    rad_vel = rad_sample[3]
    rad_w = (rad_sample[4] + 0.15)
    x_rad, y_rad, azi_rad, del_rad_x, del_rad_y, del_rad_a, vx_rad, vy_rad = estimations.update_radar_pos(x_rad, y_rad, azi_rad,
                                                                                          rad_vel, rad_w, time_diff)
    radar_velocities.append(rad_vel)

    radar_azimuths.append(np.rad2deg(azi_rad))
    riss_azimuths.append(azi)
    # compute Z matrix: subtract x,y,vx,vy,Azimuth
    # Z = [x_pos - rad_sample[1], y_pos - rad_sample[2], vx - rad_sample[3],
    # vy - rad_sample[4], azi - rad_sample[5]]

    # z_diff = [del_x - del_rad_x, del_y - del_rad_y, vx - rad_sample[3], vy - rad_sample[4], del_a - del_rad_a]
    azi_diff_z = azi-np.rad2deg(azi_rad)
    if azi_diff_z > 180:
        azi_diff_z = azi_diff_z - 360
    if azi_diff_z < -180:
        azi_diff_z = azi_diff_z + 360
    # Z_w = [0, 0, vx-vx_rad, vy-vy_rad, azi_diff_z]
    Z_w = [x_pos - x_rad, y_pos - y_rad, vx - vx_rad, vy - vy_rad, azi_diff_z]
    # Z_w = [velocity - rad_vel, wz-rad_w]
    # Z_w = [x_pos - x_rad, y_pos - y_rad, vx - vx_rad, vy - vy_rad, wz-rad_w, azi - np.rad2deg(azi_rad)]

    # NO X Y IN MEASUREMENT
    # Z_w = [vx - vx_rad, vy - vy_rad, wz-rad_w]

    vx_rad_array.append(vx_rad), vy_rad_array.append(vy_rad)
    vx_riss.append(vx), vy_riss.append(vy)

    radar_w.append(rad_w), riss_w.append(wz)
    z_vectors.append(Z_w)
    # send Z matrix into KF with time_diff, azi and acceleration from odometer
    # direction_flag = data_rad1[i_rad - 1, 7]
    num_inliers1 = rad_sample[5]
    num_inliers2 = rad_sample[6]
    # num_inliers = data_rad1[i_rad - 1, 6]

    kf.tune_R(None, num_inliers1, num_inliers2)
    # start_out, end_out = kf.handle_outage(num_inliers1, num_inliers2, time)
    start_out, end_out = kf.handle_outage_with_duration_constraint(num_inliers1, num_inliers2, time)
    is_outage = kf.is_outage()
    corrections, P = kf.update(Z_w, azi, accel, time_diff, pitch, num_inliers1, num_inliers2)

    x_pos_errors.append(corrections[0])
    y_pos_errors.append(corrections[1])

    correction_data.append(corrections)

    P_set1.append(P[0, 0])
    P_set2.append(P[1, 1])
    P_set3.append(P[2, 2])
    P_set4.append(P[3, 3])
    P_set5.append(P[4, 4])
    P_set6.append(P[5, 5])
    P_set7.append(P[6, 6])
    P_set8.append(P[7, 7])
    P_set9.append(P[8, 8])

    # print(corrections)
    x_pos = x_pos - corrections[0]
    y_pos = y_pos - corrections[1]
    azi = azi - corrections[6]
    # print(corrections[6])
    # h is position 2
    corr_vx = corrections[3]
    corr_vy = corrections[4]
    # vu is position 5
    # acceleration = acceleration - corrections[7]
    # omega = omega - corrections[8]

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
    if is_outage:
        outage_positions.append((x_pos, y_pos))
        out_flag.append(1)
    if not is_outage:
        out_flag.append(0)
    if start_out == 1:
        outage_starts.append((x_pos, y_pos))
    if end_out:
        outage_starts.append((x_pos, y_pos))

    array = np.asarray(ref_sol[1:, 0])
    idx = (np.abs(array - time)).argmin()
    reference_x.append(ref_sol[idx, 3])
    reference_y.append(ref_sol[idx, 4])

print(np.shape(reference_x))
outage_starts = np.array(outage_starts)
print('start outages', np.shape(outage_starts))
outage_ends = np.array(outage_ends)
outage_positions = np.array(outage_positions)

confidence_list = kf.get_confidence_list()
outage_times = np.array(kf.get_outages())
correction_data = np.array(correction_data)
distance_travelled = np.sum(ref_sol[1:, 1])*(50*0.001)
print('distance travelled', distance_travelled)
z_vectors = np.array(z_vectors)

print('fusion results')
fused_data = np.column_stack((time_stamps[1:], odo_vel, riss_w, corrected_x, corrected_y))
print(np.shape(fused_data))

pos2d_rms, pos2d_max, x_rms, x_max, y_rms, y_max, v_rms, v_max, w_rms, w_max = trajectory_errors.get_errors(ref_sol, fused_data)

total_time = (time_stamps[-1]-time_stamps[0])/(1000 * 60) # time in minutes
print('total time:', total_time, 'minutes')
print('fusion position 2d rms:', pos2d_rms)
print('fusion position 2d max:', pos2d_max)

print('x rms:', x_rms)
print('x max:', x_max)
print('y rms:', y_rms)
print('y max:', y_max)
print('v rms:', v_rms)
print('v max:', v_max)
print('w rms:', w_rms)
print('w max:', w_max)
print('percentage of fusion position error to distance travelled', (pos2d_rms/distance_travelled)*100)
print('percentage of fusion position error to distance travelled', (pos2d_max/distance_travelled)*100)
print('percentage of fusion position error to distance travelled', (x_rms/distance_travelled)*100)
print('percentage of fusion position error to distance travelled', (x_max/distance_travelled)*100)
print('percentage of fusion position error to distance travelled', (y_rms/distance_travelled)*100)
print('percentage of fusion position error to distance travelled', (y_max/distance_travelled)*100)




print('riss alone results')
riss_data = np.column_stack((time_stamps[1:], odo_vel, riss_w, unt_x, unt_y))
pos2d_rms, pos2d_max, x_rms, x_max, y_rms, y_max, v_rms, v_max, w_rms, w_max = trajectory_errors.get_errors(ref_sol, riss_data)
print('total time:', total_time, 'minutes')
print('riss position 2d rms:', pos2d_rms)
print('riss position 2d max:', pos2d_max)
print('x rms:', x_rms)
print('x max:', x_max)
print('y rms:', y_rms)
print('y max:', y_max)
print('v rms:', v_rms)
print('v max:', v_max)
print('w rms:', w_rms)
print('w max:', w_max)

#
print('fusion percentage errors')
e_time, e_x_fusion, e_y_fusion, absolute_errors = trajectory_errors.errors_v_time(ref_sol, fused_data)
percent_sub50cm, percent_sub1m, percent_sub1_5m, percent_sub2m = trajectory_errors.percentage_analysis(absolute_errors)
print('percent of time error < 50 cm:', percent_sub50cm)
print('percent of time error < 1 m:', percent_sub1m)
print('percent of time error < 1.5 m:', percent_sub1_5m)
print('percent of time error < 2 m:', percent_sub2m)

print('RISS alone percentage errors')
e_time, e_x, e_y, absolute_errors = trajectory_errors.errors_v_time(ref_sol, riss_data)
percent_sub50cm, percent_sub1m, percent_sub1_5m, percent_sub2m = trajectory_errors.percentage_analysis(absolute_errors)
print('percent of time error < 50 cm:', percent_sub50cm)
print('percent of time error < 1 m:', percent_sub1m)
print('percent of time error < 1.5 m:', percent_sub1_5m)
print('percent of time error < 2 m:', percent_sub2m)


####### PLOTS #################
## Kalman filter, riss and radar plots
plt.figure(1)
plt.plot(corrected_x, corrected_y, 'r', label='RISS/Radar')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.plot(ref_sol[:, 3], ref_sol[:, 4], 'k', label = 'Reference Trajectory')

# plt.plot(riss_results[:, 1], riss_results[:, 2], label='riss')
# plt.plot(radar[:,1],radar[:,2],label='radar')
# plt.plot(radar_x, radar_y, label='radar')
# plt.plot(inc_x, inc_y, label='incremental riss test')
#
# #plt.plot(radar_x, radar_y, label='Radar')
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
# # plt.grid()
# plt.legend()
# plt.ylabel('Y Position (m)')
# plt.xlabel('X Position (m)')
#
# plt.figure(2)
# plt.plot(unt_x, unt_y, 'r', label='RISS')
# plt.plot(ref_sol[:, 3], ref_sol[:, 4], 'k', label = 'Reference Trajectory')
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
# plt.legend()
# # plt.plot(x_pos_errors)
# # plt.show()
#
# plt.figure(20)
# plt.plot(radar_x, radar_y, 'r', label='Radar')
# plt.plot(ref_sol[:, 3], ref_sol[:, 4], 'k', label = 'Reference Trajectory')
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
# plt.legend()
#
# plt.figure(3)
# plt.plot(vx_riss, label='vx riss/radar')
# plt.plot(vx_rad_array, label='vx radar')
# plt.plot(untouched_vx, label='vx riss standalone')
#
# plt.legend()
#
# plt.figure(4)
# plt.plot(vy_riss, label='vy riss/radar')
# plt.plot(vy_rad_array, label='vy radar')
# plt.plot(untouched_vy, label='vy riss standalone')
# plt.legend()
#
# plt.figure(5)
# plt.plot(riss_w, label='w riss')
# plt.plot(radar_w, label='w radar')
# plt.legend()
#
# plt.figure(6)
# plt.plot(ref_sol[:, 3], ref_sol[:, 4], 'k', label = 'Reference Trajectory')
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
# plt.legend()
#
# plt.figure(7)
# plt.plot(radar_data[:2000, 4], '.', label='inliers 1')
# plt.legend()
#
# plt.figure(8)
# plt.plot(radar_data[:2000, 9], '.', label='inliers 2')
# plt.legend()
#
# plt.figure(16)
# plt.plot(radar_azimuths, label='radar azimuths')
# plt.plot(riss_azimuths, label='riss azimuths')
# plt.legend()
#
# # plt.figure(11)
# # plt.plot(z_vectors[:, 0], label='z x')
# # plt.legend()
# # plt.figure(14)
# # plt.plot(z_vectors[:, 1], label='z y')
# # plt.legend()
# # plt.figure(15)
# # plt.plot(z_vectors[:, 2], label='z vx')
# # plt.legend()
# # plt.plot(z_vectors[:, 3], label='z vy')
# # plt.legend()
# # plt.figure(17)
# # plt.plot(z_vectors[:, 4], label='z azimuth')
# # plt.legend()
#
# # plt.figure(9)
# # plt.plot(P_set1, label='x')
# # plt.plot(P_set2, label='y')
# # plt.plot(P_set3, label='z')
# # plt.plot(P_set4, label='vx')
# # plt.plot(P_set5, label='vy')
# # plt.plot(P_set6, label='vz')
# # plt.plot(P_set7, label='A')
# # plt.plot(P_set8, label='a_odo')
# # plt.plot(P_set9, label='w')
# # plt.legend()
#
# plt.figure(10)
# plt.plot(correction_data[:, 3], label='vx corrections')
# plt.plot(correction_data[:, 4], label='vy corrections')
# plt.legend()
#
# plt.figure(12)
# # plt.plot(outage_times[:, 0], outage_times[:, 1], label='outages')
# plt.plot(outage_times[:, 1], label='outages')
# plt.legend()
#
# plt.figure(13)
# plt.plot(corrected_x, corrected_y, label='RISS/Radar')
# plt.plot(unt_x, unt_y, label='RISS')
# plt.plot(radar_x, radar_y, label='Radar')
# plt.plot(outage_positions[:, 0], outage_positions[:, 1], 'r.')
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
# plt.legend()
# plt.ylabel('Y Position (m)')
# plt.xlabel('X Position (m)')
#
# # plt.figure(16)
# # plt.plot(correction_data[:, 8], label='w corrections')
# # plt.legend()
# #
# # plt.figure(17)
# # plt.plot(odo_vel, 'b.')
#
# ##plots of error vs time to see where error is greatest
#
e_x_sq_fusion = [i**2 for i in e_x_fusion]
e_y_sq_fusion = [i**2 for i in e_y_fusion]
print(np.shape(e_x_sq_fusion))
lis = [e_x_sq_fusion, e_y_sq_fusion]
e_sum_fusion = [sum(x) for x in zip(*lis)]
print(np.shape(e_sum_fusion))
e_sq_fusion = [np.sqrt(i) for i in e_sum_fusion]

e_x_sq = [i**2 for i in e_x]
e_y_sq = [i**2 for i in e_y]
print(np.shape(e_x_sq_fusion))
lis = [e_x_sq, e_y_sq]
e_sum = [sum(x) for x in zip(*lis)]
print(np.shape(e_sum))
e_sq = [np.sqrt(i) for i in e_sum]

time_stamps = [i/1000 - 3132.062 for i in time_stamps]
print(time_stamps[0])

plt.figure(18)
plt.plot(time_stamps[0:1908], e_sq, label='RISS Position Error (m)')
plt.plot(time_stamps[0:1908], e_sq_fusion, '--', label='RISS/ESR Position Error(m)')
plt.xlabel('Time (seconds)')
plt.ylabel('Error (meters)')
plt.grid()
plt.legend()
# #
# # plt.figure(19)
# # plt.plot(e_x, label='x error')
# # plt.plot(e_y, label='y error')
# # plt.plot(outage_times[:, 1], label='outages')
# # plt.legend()
#
plt.show()

# animation

# y_ref = synched_data[1:-1, 4]
# x_rad = synched_data[1:-1, 18]
# y_rad = synched_data[1:-1, 19]

#### ANIMATION SEGMENT
#
# # setup animated dot arrays
# outage_array = []
# fusion_array = []
# button_x = 75
# button_y = 82
# for i in range(len(out_flag)):
#     if out_flag[i]:
#         outage_array.append([button_x, button_y])
#         fusion_array.append([np.nan, np.nan])
#     else:
#         outage_array.append([np.nan, np.nan])
#         fusion_array.append([button_x, button_y])
# #
# fig, ax = plt.subplots()
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
# plt.text(button_x + 4, button_y - 1,'Radar Updates')
#
# # line_rad, = ax.plot(radar_x, radar_y, color='b', label = 'radar')
# # line_riss, = ax.plot(unt_x, unt_y, color='g', label='RISS')
# line_fusion, = ax.plot(corrected_x, corrected_y, color='b', label='RISS/radar')
# line_ref, = ax.plot(reference_x, reference_y, color='r', label = 'reference')
# radar_button_on, = ax.plot(fusion_array[:][0], fusion_array[:][1], 'o', color='g')
# radar_button_off, = ax.plot(outage_array[:][0], outage_array[:][1], 'o', color='r')
# plt.legend()
# def update(num, x, y, line):
#     # line_radar.set_data(radar_x[:num], radar_y[:num])
#     #line_riss.set_data(unt_x[:num], unt_y[:num])
#     line_fusion.set_data(corrected_x[:num], corrected_y[:num])
#     line_ref.set_data(reference_x[:num],  reference_y[:num])
#     radar_button_on.set_data(fusion_array[num][0], fusion_array[num][1])
#     radar_button_off.set_data(outage_array[num][0], outage_array[num][1])
#     return line_fusion, line_ref, radar_button_on, radar_button_off,
#
# ani = animation.FuncAnimation(fig, update, len(radar_x), fargs=[corrected_x, corrected_y, line_fusion],
#                               interval=5, blit=False)
# plt.show()
# # ani.save('test.gif')
