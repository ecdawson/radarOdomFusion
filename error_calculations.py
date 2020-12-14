import numpy as np

############################# Error Parameter ################
def error_parameter(solution, reference):
    error_vector = [a_i - b_i for a_i, b_i in zip(reference, solution)]
    # (np.array(reference) - np.array(solution))
    # print ("error is : ", error)
    # print(np.shape(error_vector))
    error_squared = map(lambda x: x ** 2, error_vector)
    error_squared = list(error_squared)
    # print("error_squared is : ", error_squared)
    rms_error = np.sqrt(np.mean(error_squared))
    max_error = np.max(np.absolute(error_vector))
    return rms_error, max_error, error_vector


############################ Velocity Error ################
def velocity_components_error(vn, ve, ref_vn, ref_ve):
    vn_rms, vn_max, _ = error_parameter(vn, ref_vn)
    ve_rms, ve_max, _ = error_parameter(ve, ref_ve)
    return vn_rms, vn_max, ve_rms, ve_max

def velocity_error(v, ref_vel):
    v_rms, v_max, _ = error_parameter(v, ref_vel)
    return v_rms, v_max


############################ 2D Position Error ################

def position_error(x_meters, y_meters, ref_x_meters, ref_y_meters):
    x_rms, x_max, _ = error_parameter(x_meters, ref_x_meters)
    y_rms, y_max, _ = error_parameter(y_meters, ref_y_meters)

    POS2D_RMS = np.sqrt((x_rms ** 2) + (y_rms ** 2))
    POS2D_MAX = np.sqrt((x_max ** 2) + (y_max ** 2))
    return POS2D_RMS, POS2D_MAX, x_rms, x_max, y_rms, y_max

def error_notRMS(solution, reference):
    error = [a_i - b_i for a_i, b_i in zip(solution, reference)]
    return error

