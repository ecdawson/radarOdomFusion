import numpy as np
import error_calculations as ec
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def get_errors(ref_sol, rad_sol):
    # time, 'forward_velocity', 'angular_velocity', 'x_position', 'y_position'
    # ref_sol = ref_sol

    # time, x, y, v, w
    # rad_sol = rad_sol
    # print(np.shape(ref_sol[2, 1:]))

    full_array = []
    i_rad = 1
    i_ref = 1
    times = []
    refs = []
    rads = []

    for k in range(0, 4000):
        t_ref = ref_sol[i_ref, 0]
        t_rad = rad_sol[i_rad, 0]
        time_diff = t_ref - t_rad
        if t_ref == t_rad:
            # get next time
            t_cur = t_ref
            ref = ref_sol[i_ref, 1:]
            rad = rad_sol[i_rad, 1:]
            # times.append(t_cur)
            # refs.extend(ref)
            # rads.extend(rad)
            # full_array.extend([t_cur, ref, rad])
            full_array.append(np.hstack((t_cur, ref, rad)))
            i_ref = i_ref + 1
            i_rad = i_rad + 1

        if t_ref < t_rad:
            # get next time
            t_cur = t_ref
            ref = ref_sol[i_ref, 1:]
            rad = [np.nan, np.nan, np.nan, np.nan, np.nan]
            # times.append(t_cur)
            # refs.extend(ref)
            # rads.extend(rad)
            # full_array.append(np.hstack((t_cur, ref, rad)))
            i_ref = i_ref + 1

        if t_ref > t_rad:
            t_cur = t_rad
            ref = [np.nan, np.nan, np.nan, np.nan, np.nan]
            rad = rad_sol[i_rad, 1:]
            # times.append(t_cur)
            # refs.extend(ref)
            # rads.extend(rad)
            # full_array.append(np.hstack((t_cur, ref, rad)))
            i_rad = i_rad + 1

    # print(np.shape(refs))
    # print(np.shape(rads))
    # full_array = np.hstack((times, refs, rads))
    # time, v, w, x, y, A, sol_x, sol_y, sol_v, sol_w
    # full_array = np.array(full_array)
    # full_array.reshape(5000, 11)
    # print(full_array)
    # print('full array:', np.shape(full_array))

    df = pd.DataFrame(full_array)
    # print(df.isna().sum())
    # df = df.dropna(how='any')
    # print(np.shape(df[0][:]))
    data = np.array(df)
    print(np.shape(data))
    print(data[1000, :])

    # data = full_array

    # # print(np.shape(data))

    # position_error(x, y, x_ref, y_ref)

    # data: time, 'forward_velocity','angular_velocity','x_position','y_position','azimuth', odo_vel, riss_w, corrected_x, corrected_y
    pos2d_rms, pos2d_max, x_rms, x_max, y_rms, y_max = ec.position_error(data[:, 8], data[:, 9], data[:, 3], data[:, 4])

    # velocity RMS (v, ref_v)
    v_rms, v_max = ec.velocity_error(data[:, 6], data[:, 1])
    w_rms, w_max = ec.velocity_error(data[:, 7], data[:, 2])

    return pos2d_rms, pos2d_max, x_rms, x_max, y_rms, y_max, v_rms, v_max, w_rms, w_max


def errors_v_time(ref_sol, rad_sol):
    full_array = []
    i_rad = 1
    i_ref = 1
    time = []

    for k in range(0, 4000):
        t_ref = ref_sol[i_ref, 0]
        t_rad = rad_sol[i_rad, 0]
        time_diff = t_ref - t_rad
        if t_ref == t_rad:
            # get next time
            t_cur = t_ref
            time.append(t_cur)
            ref = ref_sol[i_ref, 1:]
            rad = rad_sol[i_rad, 1:]
            full_array.append(np.hstack((t_cur, ref, rad)))
            i_ref = i_ref + 1
            i_rad = i_rad + 1

        if t_ref < t_rad:
            i_ref = i_ref + 1

        if t_ref > t_rad:
            i_rad = i_rad + 1

    df = pd.DataFrame(full_array)
    data = np.array(df)
    x_errors = ec.error_notRMS(data[:, 8], data[:, 3])
    y_errors = ec.error_notRMS(data[:, 9], data[:, 4])

    x_sq = list(map(lambda x: x ** 2, x_errors))
    y_sq = list(map(lambda x: x ** 2, y_errors))
    sum_xy = list(map(lambda x, y: x + y, x_sq, y_sq))
    distance_errors = list(map(lambda x: np.sqrt(x), sum_xy))

    return time, x_errors, y_errors, distance_errors


def percentage_analysis(errors):
    # for percentage of time that error is < 50cm, 1m, 1.5m, 2m

    count_sub50cm = 0
    count_sub1m = 0
    count_sub1_5m = 0
    count_sub2m = 0
    total = len(errors)

    for i in errors:
        if i <= 0.5:
            count_sub50cm = count_sub50cm + 1
        if i <= 1:
            count_sub1m = count_sub1m + 1
        if i <= 1.5:
            count_sub1_5m = count_sub1_5m + 1
        if i <= 2:
            count_sub2m = count_sub2m + 1

    percent_sub50cm = (count_sub50cm/total)*100
    percent_sub1m = (count_sub1m / total) * 100
    percent_sub1_5m = (count_sub1_5m / total) * 100
    percent_sub2m = (count_sub2m / total) * 100

    return percent_sub50cm, percent_sub1m, percent_sub1_5m, percent_sub2m