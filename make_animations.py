import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import io

# 3 and 4 are reference x and y, 18 and 19 for radar x and y
synched_data = np.genfromtxt('trajectory3_analysis_test.csv', delimiter=',')
floor_map = io.loadmat('floor_map.mat')
synched_data = np.array(synched_data)
print(np.shape(synched_data))

x_ref = synched_data[1:-1, 3]

y_ref = synched_data[1:-1, 4]
x_rad = synched_data[1:-1, 18]
y_rad = synched_data[1:-1, 19]

print(np.count_nonzero(np.isnan(y_ref)))

fig, ax = plt.subplots()
# plt.plot(floor_map['floor_map_pcl'][:, 0], floor_map['floor_map_pcl'][:, 1], 'b,')
line_rad, = ax.plot(x_rad, y_rad, color='k')
line_ref, = ax.plot(x_ref, y_ref, color='r')

def update(num, x, y, line):
    line_rad.set_data(x_rad[:num], y_rad[:num])
    line_ref.set_data(x_ref[:num], y_ref[:num])
    return line_ref, line_rad,

ani = animation.FuncAnimation(fig, update, len(x_rad), fargs=[x_ref, y_ref, line_ref],
                              interval=5, blit=False)
plt.show()
ani.save('test.gif')


