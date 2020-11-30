import csv
import ransac
import numpy as np
import math
import matplotlib.pyplot as plt
import estimations
import dataStruct
from riss import riss


########import data########################
#indices: timestamp = 17, ID = 1, range = 4, velocity = 5, azimuth = 6

data12 = np.genfromtxt('#1#2_PointCloudCAN.csv', delimiter=',')
data34 = np.genfromtxt('#3#4_PointCloudCAN.csv', delimiter=',')

######## mounting information and transformation matrices##############

#mouting angles and coordinates (metres)
#radar measures theta the opposite way from convention, ie z is down so have changed this here as well as sign of angles for velocity estimation
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
      [0,0,1]]
   
T2 = [[np.cos(theta2), np.sin(theta2), x2], 
      [-np.sin(theta2), np.cos(theta2), y2],
      [0,0,1]]
T3 = [[np.cos(theta3), np.sin(theta3), x3], 
      [-np.sin(theta3), np.cos(theta3), y3],
      [0,0,1]]
T4 = [[np.cos(theta4), np.sin(theta4), x4], 
      [-np.sin(theta4), np.cos(theta4), y4],
      [0,0,1]]


#separate data by radar ID. separate_IDs also converts all angles to radians. 
radar1,radar2 = dataStruct.separate_IDs(data12, 1, 2)
radar3,radar4 = dataStruct.separate_IDs(data34, 3, 4)

#calculate x and y detection positions and append them to the data. all points are in vehicle frame, not sensor frame

rad1 = dataStruct.calc_target_positions(radar1,T1)
rad2 = dataStruct.calc_target_positions(radar2,T2)
rad3 = dataStruct.calc_target_positions(radar3,T3)
rad4 = dataStruct.calc_target_positions(radar4,T4)


#partition data so accessible by time

dat1 = dataStruct.time_structure(rad1)
dat2 = dataStruct.time_structure(rad2)
dat3 = dataStruct.time_structure(rad3)
dat4 = dataStruct.time_structure(rad4)





#
#
# #cut data so that all radar measurements start and end at same time
# sync1 = dat1[0:3474,:]
# sync2 = dat2[0:3474,:]
# sync3 = dat3[39:len(dat3[:,0])-1,:]
# sync4 = dat4[39:len(dat4[:,0])-1,:]


#
# #############velocity estimation ################### 1 means rear radar
# w_vels1,fwd_vels1,ref_speed1,ref_w1,time1,inliers1,in_time1,scan_len1,reverse_flag1, ave_ranges1 = estimations.get_vels(dat1,theta1,x1,y1,0)
# w_vels2,fwd_vels2,ref_speed2,ref_w2,time2,inliers2,in_time2,scan_len2,reverse_flag2,ave_ranges2 = estimations.get_vels(dat2,theta2,x2,y2,0)
w_vels3,fwd_vels3,ref_speed3,ref_w3,time3, inliers3,in_time3,scan_len3,reverse_flag3, ave_ranges3 = estimations.get_vels(dat3,theta3,x3,y3,1)
w_vels4,fwd_vels4,ref_speed4,ref_w4,time4, inliers4,in_time4,scan_len4,reverse_flag4, ave_ranges4 = estimations.get_vels(dat4,theta4,x4,y4,1)
#
# #print(inliers1)
#
# print(np.shape(w_vels1))
# print(np.shape(fwd_vels1))
# print(np.shape(time1))
# print(np.shape(inliers1))
# print(np.shape(in_time1))
# print(np.shape(scan_len1))
# print(np.shape(reverse_flag1))
#
# print(type(w_vels2))
# print(type(fwd_vels2))
# print(type(time2))
# print(type(inliers2))
# print(type(in_time2))
# print(type(scan_len2))
# print(type(reverse_flag2))
#
#
# np.savetxt('vel_estimation_R1.csv', (time1,fwd_vels1, w_vels1, ref_speed1,ref_w1,scan_len1,inliers1,reverse_flag1, ave_ranges1), delimiter=',')
# np.savetxt('vel_estimation_R2.csv', (time2,fwd_vels2, w_vels2, ref_speed2,ref_w2,scan_len2,inliers2,reverse_flag2, ave_ranges2), delimiter=',')
np.savetxt('vel_estimation_R3.csv', (time3,fwd_vels3, w_vels3, ref_speed3,ref_w3,scan_len3,inliers3,reverse_flag3, ave_ranges3), delimiter=',')
np.savetxt('vel_estimation_R4.csv', (time4,fwd_vels4, w_vels4, ref_speed4,ref_w4,scan_len4,inliers4,reverse_flag4, ave_ranges4), delimiter=',')
#
#
#
#
#
# plt.figure(1)
# plt.plot(time1,fwd_vels1, label = 'estimated velocity radar1')
# plt.plot(time2,fwd_vels2, label = 'estimated velocity radar2')
# #plt.plot(time3,fwd_vels3, label = 'estimated velocity radar3')
# #plt.plot(time4,fwd_vels4, label = 'estimated velocity radar3')
#
#
# #plt.plot(time1,ref_speed1,label = 'reference velocity')
# plt.ylabel('Velocity (m/s)')
# plt.xlabel('Timestamp (ms)')
# plt.legend()
#

# plt.figure(2)
# plt.plot(time1,w_vels1,label = 'estimated velocity radar1')
# plt.plot(time2,w_vels2,label = 'estimated velocity radar2')
# #plt.plot(time4,w_vels4,label = 'estimated velocity radar4')
# #plt.plot(time1,ref_w1,label = 'reference velocity')
# plt.ylabel('Angular Velocity (deg/s)')
# plt.xlabel('Timestamp (ms)')
# plt.legend()
# plt.show()
#
#plt.figure(3)
#plt.plot(in_time1,inliers1,'b.',label = 'inliers radar1')
# plt.plot(in_time2,inliers2,'r.',label = 'inliers radar2')
# #plt.plot(in_time4,inliers4,'g.',label = 'inliers radar4')
# plt.ylabel('Number of Inliers')
# plt.xlabel('Timestamp (ms)')
# plt.legend()
# plt.show()
#
##plt.figure(4)
# # plt.plot(in_time4,inliers4,'b.',label = 'inliers radar4')
# # plt.ylabel('Number of Inliers')
# # plt.xlabel('Timestamp (ms)')
# # plt.legend()
# plt.show()
