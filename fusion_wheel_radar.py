import numpy as np
import matplotlib.pyplot as plt
from scipy import io
from scipy import signal
from data_synchronization import data_synch
from wheel_tex import wheel_sensor
import estimations
import trajectory_errors

floor_map = io.loadmat('floor_map.mat')
radar_data = np.genfromtxt('r1r2estimates.csv', delimiter=',')
ref_sol = np.genfromtxt('ref_generated.csv', delimiter=',')

wheel_tex = np.genfromtxt('#3#4_PointCloudCAN.csv', delimiter=',')
# wheel_tex = wheel_tex[:160000, :]
# rad dat time, x, y, v, w, inliers1, inliers2
rad_dat = np.column_stack((radar_data[1:-1, 0], radar_data[1:-1, 11], radar_data[1:-1, 12], radar_data[1:-1, 9],
                           radar_data[1:-1, 10], radar_data[1:-1, 4], radar_data[1:-1, 8]))

filt_w = signal.savgol_filter(-wheel_tex[1:-1, 10], 39, 3)
wheel_tex_dat = np.column_stack((wheel_tex[1:-1, -1], wheel_tex[1:-1, 9], filt_w))

x_m = 64.298
y_m = -74.623
azimuth_start = 0

x_rad = x_m
y_rad = y_m
azi_rad = azimuth_start

time_stamps = []
wheel_x = []
wheel_y = []
rad_x_positions = []
rad_y_positions = []
v_wheels = []
wz_wheels = []
v_radars = []
wz_radars = []
azi_rads = []
azi_wheels = []

i_wheel = 0
i_rad = 0
wheel_solution = wheel_sensor(azimuth_start, x_m, y_m)

synch_for_fusion = data_synch(wheel_tex_dat, rad_dat)
rad_sample, imu_sample, i_rad, i_wheel, time = synch_for_fusion.get_synched_samples(i_rad, i_wheel)

previous_time = time
time_stamps.append(time)

for i in range(1, 5000):
    rad_sample, wheel_sample, i_rad, i_wheel, time = synch_for_fusion.get_synched_samples(i_rad, i_wheel)
    w_wheel = wheel_sample[2]
    v_wheel = wheel_sample[1]
    v_radar = rad_sample[3]
    w_radar = rad_sample[4]
    w_radar = w_radar - 0.65

    next_time = time
    time_stamps.append(time)
    v_wheels.append(v_wheel)
    v_radars.append(v_radar)
    wz_wheels.append(w_wheel)
    wz_radars.append(w_radar)

    i_rad = i_rad + 1
    i_wheel = i_wheel + 1

    gap, time_diff = synch_for_fusion.is_gap(previous_time, next_time)
    time_diff = time_diff / 1000

    # wheel tex update
    x_pos, y_pos, azimuth, vx, vy = wheel_solution.update(time_diff, v_wheel, w_wheel)
    wheel_x.append(x_pos)
    wheel_y.append(y_pos)
    azi_wheels.append(azimuth)

    # radar update
    x_rad, y_rad, azi_rad, _, _, _ = estimations.update_radar_pos(x_rad, y_rad, azi_rad, v_radar, w_radar, time_diff)
    rad_x_positions.append(x_rad)
    rad_y_positions.append(y_rad)
    azi_rads.append(azi_rad)

    previous_time = next_time

# Error Calculations
#RMS
wheel_results = np.column_stack((time_stamps[1:], v_wheels, wz_wheels, wheel_x, wheel_y))
radar_results = np.column_stack((time_stamps[1:], v_radars, wz_radars, rad_x_positions, rad_y_positions))

pos2d_rms_wheel, pos2d_max_wheel, x_rms_wheel, x_max_wheel, y_rms_wheel, y_max_wheel, v_rms_wheel, v_max_wheel, \
    w_rms_wheel, w_max_wheel = trajectory_errors.get_errors(ref_sol, wheel_results)

pos2d_rms_rad, pos2d_max_rad, x_rms_rad, x_max_rad, y_rms_rad, y_max_rad, v_rms_rad, v_max_rad, \
    w_rms_rad, w_max_rad = trajectory_errors.get_errors(ref_sol, radar_results)

total_time = (time_stamps[-1] - time_stamps[0]) / (1000 * 60)  # time in minutes
print('total time:', total_time, 'minutes')
print('Wheel Sensor Solution RMS Errors:')
print('wheel position 2d rms:', pos2d_rms_wheel)
print('wheel position 2d max:', pos2d_max_wheel)
# print('percentage of fusion position error to distance travelled', (pos2d_rms/distance_travelled)*100)
print('x rms:', x_rms_wheel)
print('x max:', x_max_wheel)
print('y rms:', y_rms_wheel)
print('y max:', y_max_wheel)
print('v rms:', v_rms_wheel)
print('v max:', v_max_wheel)
print('w rms:', w_rms_wheel)
print('w max:', w_max_wheel)

print('Radar Solution RMS Errors:')
print('Radar position 2d rms:', pos2d_rms_rad)
print('Radar position 2d max:', pos2d_max_rad)
# print('percentage of fusion position error to distance travelled', (pos2d_rms/distance_travelled)*100)
print('x rms:', x_rms_rad)
print('x max:', x_max_rad)
print('y rms:', y_rms_rad)
print('y max:', y_max_rad)
print('v rms:', v_rms_rad)
print('v max:', v_max_rad)
print('w rms:', w_rms_rad)
print('w max:', w_max_rad)

# PERCENTAGE ANALYSIS

e_time, e_x, e_y, absolute_errors = trajectory_errors.errors_v_time(ref_sol, wheel_results)
percent_sub50cm, percent_sub1m, percent_sub1_5m, percent_sub2m = trajectory_errors.percentage_analysis(absolute_errors)
print('Wheel Sensor Percentage Errors:')
print('percent of time error < 50 cm:', percent_sub50cm)
print('percent of time error < 1 m:', percent_sub1m)
print('percent of time error < 1.5 m:', percent_sub1_5m)
print('percent of time error < 2 m:', percent_sub2m)

e_time, e_x, e_y, absolute_errors = trajectory_errors.errors_v_time(ref_sol, radar_results)
percent_sub50cm, percent_sub1m, percent_sub1_5m, percent_sub2m = trajectory_errors.percentage_analysis(absolute_errors)
print('Radar Percentage Errors:')
print('percent of time error < 50 cm:', percent_sub50cm)
print('percent of time error < 1 m:', percent_sub1m)
print('percent of time error < 1.5 m:', percent_sub1_5m)
print('percent of time error < 2 m:', percent_sub2m)

# plots
plt.figure(1)
plt.plot(wheel_tex_dat[:, 0], wheel_tex_dat[:, 2], label='angular velocity m/s wheel sensor')
plt.plot(rad_dat[:, 0], rad_dat[:, 4], label='angular velocity m/s radar')
plt.legend()

plt.figure(2)
plt.plot(wheel_tex_dat[:, 0], wheel_tex_dat[:, 1], label='forward velocity m/s wheel sensor')
plt.plot(rad_dat[:, 0], rad_dat[:, 3], label='forward velocity m/s radar')
plt.legend()

plt.figure(3)
plt.plot(wheel_x, wheel_y, label='wheel sensors')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.plot(ref_sol[:5100, 3], ref_sol[:5100, 4], label='reference trajectory')
plt.legend()

plt.figure(4)
plt.plot(rad_x_positions, rad_y_positions, label='radar')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.plot(ref_sol[:5100, 3], ref_sol[:5100, 4], label='reference trajectory')
plt.legend()

plt.figure(5)
plt.plot(np.rad2deg(azi_rads), label='radar azimuth')
plt.plot(azi_wheels, label='wheel sensor azimuth')
plt.legend()

# plt.figure(6)
# plt.plot(time_stamps[1:], wz_radars, label='wz radar')
# plt.plot(ref_sol[:, 0], ref_sol[:, 2], label='wz reference')
# plt.legend()

plt.show()
