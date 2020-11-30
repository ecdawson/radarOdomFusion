from scipy import io
import numpy as np
import matplotlib.pyplot as plt


######### Floor Map ################
# header, version, globals, floor_map_pcl
floor_map = io.loadmat('floor_map.mat')
#Odometer_2 = list(Odometer_speed_2['OBDII_Speed'])
#Odometer_time_2 = list(Odometer_speed_2['OBDII_second'])

print(floor_map['floor_map_pcl'])
print(floor_map['floor_map_pcl'][0][2])
print(floor_map['floor_map_pcl'][:, 2])
print(np.shape(floor_map['floor_map_pcl']))
print(floor_map.keys())
print ("Length of Odometer 2 speed is : ",len(floor_map))

plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1],',')
plt.show()
