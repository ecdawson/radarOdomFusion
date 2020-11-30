import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import csv

# time, x, y, heading
ref_sol = np.genfromtxt('ref_generated.csv', delimiter=',')

# 'time', 'velocity_r1',' angular_vel_r1', 'num_detections1', 'num_inliers1', 'average_range1',
#           'velocity_r2', 'angular_vel_r2', 'num_detections2', 'num_inliers2', 'average_range2', 'combined_velocity',
#           'combined_angular_vel', 'x', 'y'
rad_sol = np.genfromtxt('r1r2estimates.csv', delimiter=',')

# (time_ave, both_x, both_y, filt_v_aves, w_ave, both_azi)
pos_sol = np.genfromtxt('positions_combined.csv', delimiter=',')

print(np.shape(ref_sol))
print(np.shape(rad_sol))
print(np.shape(ref_sol[2, 1:]))

full_array = []
i_rad = 1
i_ref = 1
for k in range(0, 5000):
    t_ref = ref_sol[i_ref, 0]
    t_rad = rad_sol[i_rad, 0]
    if t_ref == t_rad:
        # get next time
        t_cur = t_ref
        ref = ref_sol[i_ref, 1:]
        rad = rad_sol[i_rad, 1:]
        full_array.append(np.hstack((t_cur, ref, rad)))
        i_ref = i_ref + 1
        i_rad = i_rad + 1

    if t_ref < t_rad:
        # get next time
        t_cur = t_ref
        ref = ref_sol[i_ref, 1:]
        rad = [np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan,
               np.nan]
        full_array.append(np.hstack((t_cur, ref, rad)))
        i_ref = i_ref + 1
    if t_ref > t_rad:
        t_cur = t_rad
        ref = [np.nan, np.nan, np.nan, np.nan, np.nan]
        rad = rad_sol[i_rad, 1:]
        full_array.append(np.hstack((t_cur, ref, rad)))
        i_rad = i_rad + 1


# interpolate to fill nan places

# print(np.shape(np.hstack((t_cur, ref, rad))))

filename = 'trajectory3_analysis_test.csv'
# # fields names
fields = ['time', 'ref_vel', 'ref_angular_vel', 'ref_x_pos',
            'ref_y_pos', 'ref_azimuth', 'velocity_r1', 'angular_vel_r1', 'num_detections1', 'num_inliers1',
            'average_range1', 'velocity_r2','angular_vel_r2', 'num_detections2','num_inliers2','average_range2',
            'combined_velocities', 'combined_angular_vels', 'x', 'y']
# # writing to csv file row_no = 0

with open(filename, 'w') as csvfile:
    # creating a csv writer object
    csvwriter = csv.writer(csvfile)
    # writing the fields
    csvwriter.writerow(fields)
    row_no = 0

    for i in range(len(full_array)):
        line = full_array[i]
        row = [line[0], line[1], line[2], line[3], line[4], line[5], line[6], line[7], line[8], line[9], line[10],
               line[11], line[12], line[13], line[14], line[15], line[16], line[17], line[18], line[19]]
        row_no += 1
        csvwriter.writerow(row)
        print('Number of rows added are ' + str(row_no), end='\r')
    csvfile.close()

# error section, rms calc
# POS2D_RMS, POS2D_MAX, x_rms, x_max, y_rms, y_max = ec.position_error(, , full_array[i, 3], full_array[i, 4])

df = pd.read_csv('trajectory3_analysis_test.csv')
df.info()

df1 = pd.read_csv('trajectory3_analysis.csv')
df1.info()

full_array = np.array(full_array)
print(full_array[0, 0])
print(pos_sol[0, 0])
plt.plot(full_array[:, 1])
plt.plot(full_array[:, 16])
plt.show()