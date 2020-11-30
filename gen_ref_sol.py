import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy import signal

ref_sol = np.genfromtxt('ref_solution.csv', delimiter=',')

ref_sol[:, 3] = np.rad2deg(ref_sol[:, 3])
for i in range(len(ref_sol[:, 3])):
    if ref_sol[i, 3] < 0:
        ref_sol[i, 3] = ref_sol[i, 3] + 360

plt.plot(ref_sol[:, 3])
plt.show()
time = []
vel = []
angular_vel = []
x_pos = []
y_pos = []
azimuth = []

for i in range(len(ref_sol[:, 0])-1):
    delta_time = (ref_sol[i+1, 0]-ref_sol[i, 0])/1000
    delta_x = ref_sol[i+1, 1]-ref_sol[i, 1]
    delta_y = ref_sol[i+1, 2]-ref_sol[i, 2]
    delta_A = -(ref_sol[i + 1, 3] - ref_sol[i, 3])
    if delta_A > 5:
        delta_A = delta_A - 360
    elif delta_A < - 5:
        delta_A = delta_A + 360


    vx = delta_x/delta_time
    vy = delta_y/delta_time
    v_fwd = np.sqrt(vx**2 + vy**2)
    w = delta_A/delta_time

    time.append(ref_sol[i+1, 0])
    vel.append(v_fwd)
    angular_vel.append(w)
    x_pos.append(ref_sol[i+1, 1])
    y_pos.append(ref_sol[i+1, 2])
    azimuth.append(ref_sol[i+1, 3])

angular_vel = signal.savgol_filter(angular_vel, 41, 3)
vel = signal.savgol_filter(vel, 41, 3)

filename = 'ref_generated.csv'
# fields names
fields = ['time','forward_velocity','angular_velocity','x_position','y_position','azimuth']
# # writing to csv file
row_no = 0

with open(filename, 'w') as csvfile:
    # creating a csv writer object
    csvwriter = csv.writer(csvfile)
    # writing the fields
    csvwriter.writerow(fields)
    row_no = 0

    for i in range (len(time)):
        row = [time[i], vel[i], angular_vel[i], x_pos[i], y_pos[i], azimuth[i]]
        row_no += 1
        csvwriter.writerow(row)
        print ('Number of rows added are ' + str(row_no), end='\r')
    csvfile.close()

plt.plot(time, angular_vel)
plt.show()