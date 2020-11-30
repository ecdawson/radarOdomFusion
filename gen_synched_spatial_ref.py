import numpy as np
import csv

ref_sol = np.genfromtxt('trajectory3_analysis.csv', delimiter=',')
spatial = np.genfromtxt('Spatial_Distribution_tightened.csv', delimiter=',')


spatial = np.delete(spatial, 0, 1)
spatial = np.delete(spatial, 0, 0)

print(np.shape(ref_sol))
print(np.shape(spatial))

i_spat = 1
i_ref = 1
full_array = []

for k in range(0, 5000):
    t_ref = ref_sol[i_ref, 0]
    t_spat = spatial[i_spat, 0]
    if t_ref == t_spat:
        # get next time
        t_cur = t_ref
        ref = ref_sol[i_ref, 1:]
        spat = spatial[i_spat, 1:]
        full_array.append(np.hstack((t_cur, ref, spat)))
        i_ref = i_ref + 1
        i_spat = i_spat + 1

    if t_ref < t_spat:
        # get next time
        t_cur = t_ref
        # ref = ref_sol[i_ref, 1:]
        #
        # spat = np.ones(28)
        # spat = spat * np.nan
        # # spat = [np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan,
        #       # np.nan]
        # full_array.append(np.hstack((t_cur, ref, spat)))
        i_ref = i_ref + 1
    if t_ref > t_spat:
        t_cur = t_spat
        # ref = np.ones(19)
        # ref = ref * np.nan
        # # ref = [np.nan, np.nan, np.nan, np.nan, np.nan]
        # spat = spatial[i_spat, 1:]
        # full_array.append(np.hstack((t_cur, ref, spat)))
        i_spat = i_spat + 1

filename = 'traj3_spatialRef_tightened.csv'
# # fields names
fields = ['time', 'ref_vel', 'ref_angular_vel', 'ref_x_pos',
          'ref_y_pos', 'ref_azimuth', 'velocity_r1', 'angular_vel_r1', 'num_detections1', 'num_inliers1',
          'average_range1', 'velocity_r2','angular_vel_r2', 'num_detections2','num_inliers2','average_range2',
          'combined_velocities', 'combined_angular_vels', 'x', 'y', 'FarRightBehind',' FarRightFront' ,
          'FarRightMidFront', 'FarRightFarFront', 'MidRightBehind', 'MidRightFront', 'MidRightMidFront',
          'MidRightFarFront', 'RightBehind', 'RightFront', 'RightMidFront', 'RightFarFront', 'OverlapBehind',
          'OverlapFront', 'OverlapMidFront', 'OverlapFarFront', 'LeftBehind', 'LeftFront', 'LeftMidFront',
          'LeftFarFront', 'MidLeftBehind', 'MidLeftFront', 'MidLeftMidFront', 'MidLeftFarFront', 'FarLeftBehind',
          'FarLeftFront', 'FarLeftMidFront', 'FarLeftFarFront']
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
               line[11], line[12], line[13], line[14], line[15], line[16], line[17], line[18], line[19], line[20],
               line[21], line[22], line[23], line[24], line[25], line[26], line[27], line[28], line[29], line[30],
               line[31], line[32], line[33], line[34], line[35], line[36], line[37], line[38], line[39], line[40],
               line[41], line[42], line[43], line[44], line[45], line[46], line[47]]
        row_no += 1
        csvwriter.writerow(row)
        print('Number of rows added are ' + str(row_no), end='\r')
    csvfile.close()


print(np.shape(full_array))
print(full_array[0])