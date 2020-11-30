import numpy as np
import math
import ransac


def get_vels(data, mounting_angle, mounting_x, mounting_y, rear):
    #  angular and forward velocity estimations
    w_vels = []
    fwd_vels = []
    time = []
    ref_speed = []
    ref_w = []
    inlier_data = []
    inlier_time = []
    scan_length = []
    flag_reverse = []
    aves = []

    for i in range(len(data)):

        scan = np.array(data[i, 1])
        #  some scans in radar 3 have no data, and are filled by [0]. Catch this
        if scan is not None:
            #  more than 10 detections in a scan try to get sensor velocities
            if len(scan[:, 5]) >= 5:
                doppler_vels = scan[:, 5]
                angles = -scan[:, 6]
                angle_matrix = np.transpose([np.cos(angles), np.sin(angles)])

                #  solve for best vx and vy
                model, inliers = ransac.ransac(angle_matrix, doppler_vels, ransac.fit_with_least_squares,
                                               ransac.evaluate_model)
                if model is not None:

                    #  inliers,distances = ransac.evaluate_model(angle_matrix,doppler_vels, model, 0.1)
                    v_sensor = np.sqrt(model[0] ** 2 + model[1] ** 2)
                    alpha = np.arctan2(model[1], model[0])

                    #  calculate forward velocity and angular velocity based on vx and vy
                    v_forward = (np.cos(alpha + mounting_angle) - (mounting_y / mounting_x) * np.sin(
                        alpha + mounting_angle)) * v_sensor
                    if v_forward < 0:
                        flag = 1
                    else:
                        flag = 0

                    if rear == 0:
                        w = (np.sin(alpha + mounting_angle) / mounting_x) * v_sensor
                    else:
                        w = (np.sin(alpha + mounting_angle) / mounting_x) * v_sensor  ###!!!! figure something out

                    if w < 500:
                        w_vels.append(math.degrees(w))
                        fwd_vels.append(-v_forward)
                        time.append(scan[0, 17])
                        ref_speed.append(scan[0, 9])
                        ref_w.append(scan[0, 10])
                        scan_length.append(len(scan[:, 5]))
                        flag_reverse.append(flag)
                        inlier_data.append(inliers)
                        inlier_time.append(scan[0, 17])
                        aves.append(np.mean(scan[:, 4]))

    return w_vels, fwd_vels, ref_speed, ref_w, time, inlier_data, inlier_time, scan_length, flag_reverse, aves


def get_positions(t, v, w, x_0, y_0):
    time = t
    w_vels = w
    w_vels = np.radians(w_vels)
    fwd_vels = v
    ############# position estimation ##################

    x = np.zeros(len(w_vels))
    y = np.zeros(len(w_vels))
    azi = np.zeros(len(w_vels))
    time_return = np.zeros(len(w_vels))
    x[0] = x_0
    y[0] = y_0
    azi[0] = 0
    time_return[0] = time[0]

    for i in range(1, len(w_vels)):
        time_diff = (time[i] - time[i - 1]) / 1000
        azi[i] = azi[i - 1] + w_vels[i - 1] * time_diff
        azi[i] = azi[i] % (2 * math.pi)
        x[i] = x[i - 1] + fwd_vels[i - 1] * np.cos(azi[i - 1]) * time_diff
        y[i] = y[i - 1] + fwd_vels[i - 1] * np.sin(azi[i - 1]) * time_diff
        time_return[i] = time[i]
    return x, y, azi, time


def update_positions(x, y, azi, vel, w, time_diff):
    azi_new = azi + w * time_diff
    x_new = x + vel * np.cos(azi) * time_diff
    y_new = y + vel * np.sin(azi) * time_diff

    return x_new, y_new, azi_new


def update_radar_pos(x, y, azi, vel, w, time_diff):
    w = np.radians(w)
    vel = vel
    # print(np.rad2deg(azi))
    azi_new = azi + w * time_diff
    if azi_new >= 2 * math.pi:
        azi_new = azi_new - 2 * math.pi
    elif azi_new < 0:
        azi_new = azi_new + 2 * math.pi

    # azi_new = azi_new % (2 * math.pi)

    vx = vel * np.cos(azi)
    vy = vel * np.sin(azi)

    x_new = x + vx * time_diff
    y_new = y + vy * time_diff

    del_x = vx * time_diff
    del_y = vy * time_diff
    del_a = w * time_diff
    return x_new, y_new, azi_new, del_x, del_y, del_a, vx, vy


def gyro_azi(imu_data):
    azi = np.zeros(len(imu_data[:, 0]))
    time = np.zeros(len(imu_data[:, 0]))
    time[0] = imu_data[2, 19]
    azi[0] = 0
    w = np.radians(imu_data[:, 19])
    for i in range(3, len(imu_data[:, 0])):
        time_diff = (imu_data[i, 2] - imu_data[i - 1, 2]) / 1000
        azi[i] = azi[i - 1] + w[i - 1] * time_diff
        azi[i] = azi[i] % (-2 * math.pi)
        time[i] = imu_data[i, 2]

    return azi, time
