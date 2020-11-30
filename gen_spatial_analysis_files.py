import numpy as np
import math
import csv
import dataStruct

########import data########################
# indices: timestamp = 17, ID = 1, range = 4, velocity = 5, azimuth = 6

data12 = np.genfromtxt('#1#2_PointCloudCAN.csv', delimiter=',')
data34 = np.genfromtxt('#3#4_PointCloudCAN.csv', delimiter=',')

######## mounting information and transformation matrices##############

# mounting angles and coordinates (metres)
# radar measures theta the opposite way from convention, ie z is down so have changed this here as well as sign of
# angles for velocity estimation
theta1 = -math.radians(-43.8)
theta2 = -math.radians(44.8)
theta3 = -math.radians(-127.5)
theta4 = -math.radians(134.7)

x1 = 3.437
y1 = 0.714
z1 = 0.749

x2 = 3.437
y2 = -0.697
z2 = 0.749

x3 = -0.781
y3 = 0.775
z3 = 0.775

x4 = -0.778
y4 = -0.781
z4 = 0.779

# define transformation matrices for each sensor

T1 = [[np.cos(theta1), np.sin(theta1), x1],
      [-np.sin(theta1), np.cos(theta1), y1],
      [0, 0, 1]]

T2 = [[np.cos(theta2), np.sin(theta2), x2],
      [-np.sin(theta2), np.cos(theta2), y2],
      [0, 0, 1]]
T3 = [[np.cos(theta3), np.sin(theta3), x3],
      [-np.sin(theta3), np.cos(theta3), y3],
      [0, 0, 1]]
T4 = [[np.cos(theta4), np.sin(theta4), x4],
      [-np.sin(theta4), np.cos(theta4), y4],
      [0, 0, 1]]

# separate data by radar ID. separate_IDs also converts all angles to radians.
radar1, radar2 = dataStruct.separate_IDs(data12, 1, 2)
radar3, radar4 = dataStruct.separate_IDs(data34, 3, 4)

# calculate x and y detection positions and append them to the data. all points are in vehicle frame, not sensor frame

rad1 = dataStruct.calc_target_positions(radar1, T1)
rad2 = dataStruct.calc_target_positions(radar2, T2)
rad3 = dataStruct.calc_target_positions(radar3, T3)
rad4 = dataStruct.calc_target_positions(radar4, T4)

full_array = np.column_stack((rad1[:, 17], rad1[:, 6], rad1[:, 4], rad1[:, 5], rad1[:, 18], rad1[:, 19]))
print(np.shape(full_array))
body_range = []
body_theta = []
for i in range(len(full_array[:, 0])):
    theta = np.arctan2(full_array[i, 5], full_array[i, 4])
    r = np.sqrt(full_array[i, 5]**2 + full_array[i, 4]**2)
    body_range.append(r)
    body_theta.append(theta)

full_array = np.column_stack((full_array, body_range, body_theta))
print(np.shape(full_array))

filename = 'spatial_points1.csv'
# # fields names
fields = ['time', 'range_sensor_frame', 'angle_sensor_frame', 'doppler_vel', 'x', 'y', 'range_body_frame',
          'angle_body_frame']
# # writing to csv file row_no = 0

with open(filename, 'w') as csvfile:
    # creating a csv writer object
    csvwriter = csv.writer(csvfile)
    # writing the fields
    csvwriter.writerow(fields)
    row_no = 0

    for i in range(len(full_array)):
        line = full_array[i]
        row = [line[0], line[1], line[2], line[3], line[4], line[5]]
        row_no += 1
        csvwriter.writerow(row)
        print('Number of rows added are ' + str(row_no), end='\r')
    csvfile.close()
