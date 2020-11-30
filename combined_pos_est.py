import numpy as np
import math
from scipy import io
import estimations
import matplotlib.pyplot as plt
from riss import riss
from scipy import signal
import csv

# import data
# imported radar matrix : 4nN. time,v,w,wheel_tex_speed,wheel_tex_w
# time,v, w, _, _, scan_len, inliers, reverse_flag1, ave_len
data_rad1 = np.genfromtxt('vel_estimation_R1.csv', delimiter=',')
data_rad2 = np.genfromtxt('vel_estimation_R2.csv', delimiter=',')
ref_sol = np.genfromtxt('ref_solution.csv', delimiter=',')
imu_odo = np.genfromtxt('speed_rev_test.csv', delimiter=',')
imu_odo = np.transpose(imu_odo)

floor_map = io.loadmat('floor_map.mat')


data_rad1 = data_rad1[:, :5000]
data_rad2 = data_rad2[:, 100:5000]
# data_rad4 = np.genfromtxt('vel_estimation_R4.csv', delimiter=',')

imu = np.genfromtxt('imu.csv', delimiter=',')
speed = np.genfromtxt('vehicle_speed.csv', delimiter=',')

imu[:, 2] = imu[:, 2] / 1000000
imu = imu[:2500]
# initial pose

# based on maps from company, not initial pose file
x_m = 46.57
y_m = 30.91
azimuth_deg = np.rad2deg(-3.13629907)

# position estimation using radar data
# get_positions(time,v,w,starting_x,starting_y)
x1, y1, azi1, t_1 = estimations.get_positions(data_rad1[0, :], data_rad1[1, :], data_rad1[2, :], x_m, y_m)
x2, y2, azi2, t_2 = estimations.get_positions(data_rad2[0, :], data_rad2[1, :], data_rad2[2, :], x_m, y_m)
# x4,y4,azi4,t_4 = estimations.get_positions(data_rad4[0,:],data_rad4[1,:],data_rad4[2,:],x_m,y_m)

# azi_imu,t_imu = estimations.gyro_azi(imu)
# position estimation with wheel tex data
# x_tex,y_tex,azi_tex,t_tex = estimations.get_positions(data_rad1[0,:],data_rad1[3,:],data_rad1[4,:],x_m,y_m)

### position estimation using RISS
# time_riss,x,y,vx_riss,vy_riss,height,azi_riss,accel,roll,pitch = riss_single_floor(x_m,y_m)


filt_w = signal.savgol_filter(-data_rad1[2, :], 39, 3)
x_filt, y_filt, azi_filt, t_filt = estimations.get_positions(data_rad1[0, :], -data_rad1[1, :], filt_w, x_m, y_m)

filt_w2 = signal.savgol_filter(-data_rad2[2, :], 39, 3)
x_filt2, y_filt2, azi_filt2, t_filt2 = estimations.get_positions(data_rad2[0, :], data_rad2[1, :], filt_w2, x_m, y_m)

# calculate vx and vy for the radar estimates
vx_radar = np.cos(azi_filt) * data_rad1[1, :]
vy_radar = np.sin(azi_filt) * data_rad1[1, :]

# save position files
# time, x, y, v, w, azimuth
np.savetxt('positions_R1.csv', (t_filt, x_filt, y_filt, data_rad1[1, :], filt_w, azi_filt), delimiter=',')
np.savetxt('positions_R2.csv', (t_filt2, x_filt2, y_filt2, data_rad2[1, :], filt_w2, azi_filt2), delimiter=',')


# position estimation combining rad1 and rad2 by averaging velocities
x_ave = []
y_ave = []
w_ave = []
v_both = []
time_ave = []

azi = 0

##for i in range(len(data_rad1[0,:])):

# counters for radar1 and radar2
i1 = 0
i2 = 0

# initial x and y locations
x_prev = 0
y_prev = 0
scan_num_diff = []


# velocity sets for rad 1 and 2 are different sizes and may be missing solutions at different timestamps, depending on solutions found in extended_static_detections.py
# for each iteration, check whether there are two velocities for the given timestep (are the timestamps equal). if yes, take vels from rad 1 and 2 and average them
# if not use either the velocities from radar 1 or 2 depending which timestamp is sooner
#
to_return = []
for k in range(5000):
    t_cur1 = data_rad1[0, i1]
    t_cur2 = data_rad2[0, i2]
    if t_cur1 == t_cur2:
        # get next time
        t_cur = t_cur1
        t_next1 = data_rad1[0, i1 + 1]
        t_next2 = data_rad2[0, i2 + 1]

        if t_next1 == t_next2:
            t_next = t_next1
            if data_rad1[7,i1] == 0:
                w = filt_w2[i2 + 1]
            else:
                w = filt_w[i1 + 1]

            v = (data_rad1[1, i1 + 1] + data_rad2[1, i2 + 1]) / 2
            num_diff = data_rad1[5, i1] - data_rad2[5, i2]
            time_diff = t_next - t_cur
            # do not average the angular vel, do average the forward velocity
            i1 = i1 + 1
            i2 = i2 + 1

            v1 = data_rad1[1, i1]
            w1 = data_rad1[2, i1]
            len1 = data_rad1[5, i1]
            in1 = data_rad1[6, i1]
            ave1 = data_rad1[8, i1]
            v2 = data_rad2[1, i2]
            w2 = data_rad2[2, i2]
            len2 = data_rad2[5, i2]
            in2 = data_rad2[6, i2]
            ave2 = data_rad2[8, i2]
            to_return.append((t_next, v1, w1, len1, in1, ave1, v2, w2, len2, in2, ave2, v, w))
        elif t_next1 > t_next2:
            t_next = t_next2
            time_diff = t_next - t_cur
            w = filt_w2[i2 + 1]
            v = data_rad2[1, i2 + 1]
            i2 = i2 + 1

            v2 = data_rad2[1, i2]
            w2 = data_rad2[2, i2]
            len2 = data_rad2[5, i2]
            in2 = data_rad2[6, i2]
            ave2 = data_rad2[8, i2]
            to_return.append((t_next, np.nan, np.nan, np.nan, np.nan, np.nan, v2, w2, len2, in2, ave2, v, w))

        elif t_next1 < t_next2:
            t_next = t_next1
            time_diff = t_next - t_cur
            w = filt_w[i1 + 1]
            v = data_rad1[1, i1 + 1]
            i1 = i1 + 1

            v1 = data_rad1[1, i1]
            w1 = data_rad1[2, i1]
            len1 = data_rad1[5, i1]
            in1 = data_rad1[6, i1]
            ave1 = data_rad1[8, i1]
            v2 = np.nan
            w2 = np.nan
            len2 = np.nan
            in2 = np.nan
            ave2 = np.nan
            to_return.append((t_next, v1, w1, len1, in1, ave1, v2, w2, len2, in2, ave2, v, w))

    elif t_cur1 > t_cur2:
        t_cur = t_cur2
        t_next1 = data_rad1[0, i1 + 1]
        t_next2 = data_rad2[0, i2 + 1]
        num_diff = - data_rad2[5, i2]
        if t_next1 == t_next2:
            if data_rad1[7,i1] == 0:
                w = filt_w2[i2 + 1]
            else:
                w = filt_w[i1 + 1]
            t_next = t_next1
            time_diff = t_next - t_cur
            v = (data_rad1[1, i1 + 1] + data_rad2[1, i2 + 1]) / 2
            i1 = i1 + 1
            i2 = i2 + 1

            v1 = data_rad1[1, i1]
            w1 = data_rad1[2, i1]
            len1 = data_rad1[5, i1]
            in1 = data_rad1[6, i1]
            ave1 = data_rad1[8, i1]
            v2 = data_rad2[1, i2]
            w2 = data_rad2[2, i2]
            len2 = data_rad2[5, i2]
            in2 = data_rad2[6, i2]
            ave2 = data_rad2[8, i2]
            to_return.append((t_next, v1, w1, len1, in1, ave1, v2, w2, len2, in2, ave2, v, w))

        elif t_next1 > t_next2:
            t_next = t_next2
            time_diff = t_next - t_cur
            w = filt_w2[i2 + 1]
            v = data_rad2[1, i2 + 1]
            i2 = i2 + 1

            v2 = data_rad2[1, i2]
            w2 = data_rad2[2, i2]
            len2 = data_rad2[5, i2]
            in2 = data_rad2[6, i2]
            ave2 = data_rad2[8, i2]
            to_return.append((t_next, np.nan, np.nan, np.nan, np.nan, np.nan, v2, w2, len2, in2, ave2, v, w))

        elif t_next1 < t_next2:
            t_next = t_next1
            time_diff = t_next - t_cur
            w = filt_w[i1 + 1]
            v = data_rad1[1, i1 + 1]
            i1 = i1 + 1

            v1 = data_rad1[1, i1]
            w1 = data_rad1[2, i1]
            len1 = data_rad1[5, i1]
            in1 = data_rad1[6, i1]
            ave1 = data_rad1[8, i1]
            v2 = np.nan
            w2 = np.nan
            len2 = np.nan
            in2 = np.nan
            ave2 = np.nan
            to_return.append((t_next, v1, w1, len1, in1, ave1, v2, w2, len2, in2, ave2, v, w))

    elif t_cur1 < t_cur2:
        t_cur = t_cur1
        t_next1 = data_rad1[0, i1 + 1]
        t_next2 = data_rad2[0, i2 + 1]
        num_diff = data_rad1[5, i1]
        if t_next1 == t_next2:
            if data_rad1[7,i1] == 0:
                w = filt_w2[i2 + 1]
            else:
                w = filt_w[i1 + 1]
            t_next = t_next1
            time_diff = t_next - t_cur
            v = (data_rad1[1, i1 + 1] + data_rad2[1, i2 + 1]) / 2
            i1 = i1 + 1
            i2 = i2 + 1

            v1 = data_rad1[1, i1]
            w1 = data_rad1[2, i1]
            len1 = data_rad1[5, i1]
            in1 = data_rad1[6, i1]
            ave1 = data_rad1[8, i1]
            v2 = data_rad2[1, i2]
            w2 = data_rad2[2, i2]
            len2 = data_rad2[5, i2]
            in2 = data_rad2[6, i2]
            ave2 = data_rad2[8, i2]
            to_return.append((t_next, v1, w1, len1, in1, ave1, v2, w2, len2, in2, ave2, v, w))

        elif t_next1 > t_next2:
            t_next = t_next2
            time_diff = t_next - t_cur
            w = filt_w2[i2 + 1]
            v = data_rad2[1, i2 + 1]
            i2 = i2 + 1

            v2 = data_rad2[1, i2]
            w2 = data_rad2[2, i2]
            len2 = data_rad2[5, i2]
            in2 = data_rad2[6, i2]
            ave2 = data_rad2[8, i2]
            to_return.append((t_next, np.nan, np.nan, np.nan, np.nan, np.nan, v2, w2, len2, in2, ave2, v, w))

        elif t_next1 < t_next2:
            t_next = t_next1
            time_diff = t_next - t_cur
            w = filt_w[i1 + 1]
            v = data_rad1[1, i1 + 1]
            i1 = i1 + 1

            v1 = data_rad1[1, i1]
            w1 = data_rad1[2, i1]
            len1 = data_rad1[5, i1]
            in1 = data_rad1[6, i1]
            ave1 = data_rad1[8, i1]
            v2 = np.nan
            w2 = np.nan
            len2 = np.nan
            in2 = np.nan
            ave2 = np.nan
            to_return.append((t_next, v1, w1, len1, in1, ave1, v2, w2, len2, in2, ave2, v, w))

    if w < 15:
        w = w + 0.16
    w = np.radians(w)
    time_diff = time_diff / 1000
    scan_num_diff.append(num_diff)
    x_cur, y_cur, azi = estimations.update_positions(x_prev, y_prev, azi, v, -w, time_diff)
    x_ave.append(x_cur)
    y_ave.append(y_cur)
    w_ave.append(w)
    v_both.append(v)
    x_prev = x_cur
    y_prev = y_cur
    time_ave.append(t_next)
#    #END FOR

to_return = np.array(to_return)

## filter velocities
filt_v_aves = signal.savgol_filter(v_both, 39, 3)
both_x, both_y, both_azi, both_t = estimations.get_positions(time_ave, -filt_v_aves, np.rad2deg(w_ave), x_m, y_m)

# np.savetxt('positions_combined.csv', (time_ave, both_x, both_y, filt_v_aves, w_ave, both_azi), delimiter=',', fmt='%.2f')
#
# csv for combined estimates
filename = 'r1r2estimates.csv'
# fields names
fields = ['time', 'velocity_r1',' angular_vel_r1', 'num_detections1', 'num_inliers1', 'average_range1',
          'velocity_r2', 'angular_vel_r2', 'num_detections2', 'num_inliers2', 'average_range2', 'combined_velocity',
          'combined_angular_vel', 'x', 'y']
# # writing to csv file
row_no = 0

with open(filename, 'w') as csvfile:
    # creating a csv writer object
    csvwriter = csv.writer(csvfile)
    # writing the fields
    csvwriter.writerow(fields)
    row_no = 0

    for i in range (len(to_return[:, 0])):
        row = [to_return[i, 0], to_return[i, 1], to_return[i, 2], to_return[i, 3], to_return[i, 4], to_return[i, 5],
               to_return[i, 6], to_return[i, 7], to_return[i, 8], to_return[i, 9], to_return[i, 10], to_return[i, 11],
               to_return[i, 12], both_x[i], both_y[i]]
        row_no += 1
        csvwriter.writerow(row)
        print ('Number of rows added are ' + str(row_no), end='\r')
    csvfile.close()



# np.savetxt('r1r2estimates.csv', to_return)



# RISS implementation
riss_x = []
riss_y = []
riss = riss(0, 0, 0, np.rad2deg(-3.13629907), x_m, y_m)
for i in range(len(imu_odo)-1):
    i = i + 1
    fx = (imu_odo[i, 1])
    fy = (imu_odo[i, 2])
    wz = (imu_odo[i, 4])
    delta_time = (imu_odo[i,0]-imu_odo[i-1, 0])/1000
    gyro_bias = 0
    velocity = imu_odo[i, 5]
    x_pos, y_pos, vn, ve, height, azimuth, acc_od, pitch, roll, del_x, del_y, del_a = riss.update_metres(fx, fy, wz, delta_time, gyro_bias, velocity)
    riss_x.append(x_pos)
    riss_y.append(y_pos)
# angular velocity of rad 1 with angular velocity from IMU

plt.figure(2)
plt.plot(data_rad1[0, :], filt_w, label='Angular Velocity Estimtions Rad1')
plt.plot(data_rad2[0, :], filt_w2, label='Angular Velocity Estimtions Rad2')
plt.plot(time_ave, np.rad2deg(w_ave), label='Angular Velocity Estimtions both')
plt.plot(imu[:, 2], imu[:, 19], label='IMU angular velocities')
plt.xlabel('Time (ms)')
plt.ylabel('Angular Velocity (deg/s)')
plt.legend()


# azimuth plots
plt.figure(3)
plt.plot(t_1, np.degrees(azi1), label='azimuth Rad1')
plt.plot(t_2, np.degrees(azi2), label='azimuth Rad2')
# plt.plot(t_tex,np.degrees(azi_tex),label='azimuth Wheel Tex')
# plt.plot(t_imu[10:-1],-np.degrees(azi_imu[10:-1]),label='azimuth imu')
plt.xlabel('Time (ms)')
plt.ylabel('Estimated Azimuth (degrees)')
plt.legend()


# angular velocity plots with radar 1 filtered
# plt.figure(4)
# plt.plot(data_rad1[0,:],filt_w,label = 'Filt Angular Velocity Estimtions Rad1')
# plt.plot(data_rad2[0,:],filt_w2,label = 'Filt Angular Velocity Estimtions Rad2')
# plt.plot(imu[:,2],imu[:,19], label = 'IMU angular velocities')
# plt.xlabel('Time (ms)')
# plt.ylabel('Velocity (m/s)')
# plt.legend()
# plt.show()

# forward velocity plots

plt.figure(5)
plt.plot(data_rad1[0, :], data_rad1[1, :], label='Forward Velocity Estimations Rad1')
plt.plot(data_rad2[0, :], data_rad2[1, :], label='Forward Velocity Estimations Rad2')
plt.plot(time_ave, filt_v_aves, label='Forward Velocity Combined')
plt.plot(speed[:, 2] / 1000000, speed[:, 4], label='Forward Velocities from Odometer')
plt.xlabel('Time (ms)')
plt.ylabel('Velocity (m/s)')
plt.legend()

plt.figure(6)
plt.plot(time_ave, scan_num_diff)
plt.hlines(0, time_ave[0],time_ave[-1] , colors='r')
plt.plot(3220000, 0, 'rs')
plt.plot(3240000, 0, 'rs')

######position plots##############
# plt.plot(x1,y1, label = 'Position Estimate Front Left Radar (ID 1)')
# plt.plot(x2,y2, label = 'Position Estimate Front Right Radar (ID 2)')
# plt.plot(x_tex,y_tex, label = 'Position Estimate Wheel Tex Data')
# plt.plot(x_ave,y_ave, label = 'Combined Radars')
# plt.plot(x4,y4, label = 'Position Estimate Rear Right Radar')

# RISS PLOT
plt.figure(7)
plt.plot(ref_sol[:, 1], ref_sol[:, 2], 'r-', label='ref sol')
plt.plot(x_m,y_m,'rs')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.plot(riss_x, riss_y, label='riss')
plt.plot(both_x, both_y, label='combined radar')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')

plt.legend()

# COMBINED RADAR PLOT
plt.figure(8)
plt.plot(both_x, both_y, label='combined radar')
plt.plot(ref_sol[:, 1], ref_sol[:, 2], 'r-', label='ref sol')
plt.plot(x_m,y_m,'rs')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')

plt.legend()

# RADAR 1 PLOT
plt.figure(9)
plt.plot(x_filt, y_filt, label='radar 1 estimate')
plt.plot(ref_sol[:, 1], ref_sol[:, 2], 'r-', label='ref sol')
plt.plot(x_m,y_m,'rs')
plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')

plt.legend()

plt.show()

