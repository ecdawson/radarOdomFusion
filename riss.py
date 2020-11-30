# inertial class
import numpy as np


class riss:
    def __init__(self, latitude, longitude, height, azimuth, x_pos, y_pos):
        self.time = 0.0  # prev time

        self.fx = 0.0
        self.fy = 0.0
        self.wz = 0.0

        self.v_od = 0.0
        self.acc_od = 0.0

        self.ve = 0.0
        self.vn = 0.0
        self.vu = 0.0

        self.latitude = latitude
        self.longitude = longitude
        self.height = height

        self.x_pos = x_pos
        self.y_pos = y_pos

        self.delta_x = 0
        self.delta_y = 0
        self.delta_u = 0
        self.delta_azi = 0

        self.azimuth = azimuth

    def update(self, fx, fy, wz, delta_time, gyro_bias, velocity):
        self.fx = -fx
        self.fy = -fy
        self.wz = -wz

        R_m = 6378137.0
        R_n = 6356752.3
        g = 9.80520920998211

        delta_azimuth = (self.wz - gyro_bias) + 0.00007272 * np.sin(self.latitude) + self.ve * np.tan(self.latitude) / (
                R_n + self.height)
        # delta_azimuth = (np.rad2deg(self.wz - gyro_bias)) + np.rad2deg(0.00007272*np.sin(np.deg2rad(self.latitude)))
        # + np.rad2deg(self.ve*np.tan(np.deg2rad(self.latitude))/(R_n+self.height))
        new_azimuth = self.azimuth + (delta_azimuth * delta_time)
        # keeping azimuth between 0-360 degrees
        if new_azimuth >= 360:
            new_azimuth = new_azimuth - 360.0
        elif new_azimuth < 0:
            new_azimuth = new_azimuth + 360.0

        self.acc_od = (velocity - self.v_od) / (delta_time)
        # print "acc_od: ", self.acc_od, "fy: ", self.fy, "old V: ", self.v_od, "new V: ", new_v_od, "time_diff" ,delta_time
        self.v_od = velocity

        # pitch = np.rad2deg(np.arcsin((self.fy - self.acc_od)/g))
        # roll = -1*np.rad2deg(np.arcsin((self.fx - self.v_od*self.wz)/(g*np.cos(np.deg2rad(pitch)))))

        self.ve = self.v_od * np.sin(np.deg2rad(new_azimuth))  # * np.cos(np.deg2rad(pitch))
        self.vn = self.v_od * np.cos(np.deg2rad(new_azimuth))  # * np.cos(np.deg2rad(pitch))
        self.vu = self.v_od  # * np.sin(np.deg2rad(pitch))

        self.latitude = self.latitude + np.rad2deg(self.vn / (R_m + self.height)) * delta_time
        self.longitude = self.longitude + np.rad2deg(
            (self.ve / ((R_n + self.height) * np.cos(np.deg2rad((self.latitude)))))) * delta_time
        self.height = self.height + self.vu * delta_time
        self.azimuth = new_azimuth

        # lat , long , h, ve, vn, vu, azi, a_od, w_z [states] (pitch & roll) ?
        return self.latitude, self.longitude, self.vn, self.ve, self.height, self.azimuth

    def update_metres(self, fx, fy, wz, delta_time, gyro_bias, velocity, corr_vx, corr_vy):
        #  also want to return change in x, y Azimuth.
        self.fx = -fx
        self.fy = -fy
        self.wz = wz

        R_m = 6378137.0
        R_n = 6356752.3
        g = 9.80520920998211

        delta_azimuth = (
                self.wz - gyro_bias)  # + 0.00007272*np.sin(self.latitude) + self.ve*np.tan(self.latitude)/(R_n+self.height)
        # delta_azimuth = (np.rad2deg(self.wz - gyro_bias)) + np.rad2deg(0.00007272*np.sin(np.deg2rad(self.latitude))) + np.rad2deg(self.ve*np.tan(np.deg2rad(self.latitude))/(R_n+self.height))
        new_azimuth = self.azimuth + (delta_azimuth * delta_time)
        # keeping azimuth between 0-360 degrees
        if new_azimuth >= 360:
            new_azimuth = new_azimuth - 360.0
        elif new_azimuth < 0:
            new_azimuth = new_azimuth + 360.0

        self.acc_od = (velocity - self.v_od) / (delta_time)
        # print "acc_od: ", self.acc_od, "fy: ", self.fy, "old V: ", self.v_od, "new V: ", new_v_od, "time_diff" ,delta_time
        self.v_od = velocity

        pitch = np.rad2deg(np.arcsin((self.fy - self.acc_od) / g))
        roll = -1 * np.rad2deg(np.arcsin((self.fx - self.v_od * self.wz) / (g * np.cos(np.deg2rad(pitch)))))

        self.ve = self.v_od * np.sin(np.deg2rad(new_azimuth)) * np.cos(np.deg2rad(pitch)) - corr_vx
        self.vn = self.v_od * np.cos(np.deg2rad(new_azimuth)) * np.cos(np.deg2rad(pitch)) - corr_vy
        self.vu = self.v_od * np.sin(np.deg2rad(pitch))

        del_x = self.vn * delta_time
        del_y = self.ve * delta_time
        del_a = delta_azimuth * delta_time

        self.x_pos = self.x_pos + self.vn * delta_time
        self.y_pos = self.y_pos + self.ve * delta_time
        self.height = self.height + self.vu * delta_time
        self.azimuth = new_azimuth

        #  here vn is vx, ve is vy
        # l at , long , h, ve, vn, vu, azi, a_od, w_z [states] (pitch & roll) ?
        return self.x_pos, self.y_pos, self.vn, self.ve, self.height, self.azimuth, self.acc_od, pitch, roll, del_x, del_y, del_a

    def update_disp_meters(self, fx, fy, wz, delta_time, gyro_bias, velocity):
        #  also want to return change in x, y Azimuth.
        self.fx = -fx
        self.fy = -fy
        self.wz = wz

        R_m = 6378137.0
        R_n = 6356752.3
        g = 9.80520920998211

        bias_azimuth = (
                self.wz - gyro_bias)  # + 0.00007272*np.sin(self.latitude) +
        # self.ve*np.tan(self.latitude)/(R_n+self.height) delta_azimuth = (np.rad2deg(self.wz - gyro_bias)) +
        # np.rad2deg(0.00007272*np.sin(np.deg2rad(self.latitude))) +
        # np.rad2deg(self.ve*np.tan(np.deg2rad(self.latitude))/(R_n+self.height))

        new_azimuth = self.azimuth + (bias_azimuth * delta_time)
        # print(new_azimuth)
        delta_azimuth = bias_azimuth * delta_time
        # keeping azimuth between 0-360 degrees
        if new_azimuth >= 360:
            new_azimuth = new_azimuth - 360.0
        elif new_azimuth < 0:
            new_azimuth = new_azimuth + 360.0

        self.acc_od = (velocity - self.v_od) / delta_time
        # print "acc_od: ", self.acc_od, "fy: ", self.fy, "old V: ", self.v_od, "new V: ", new_v_od, "time_diff"
        # ,delta_time

        self.v_od = velocity

        pitch = np.rad2deg(np.arcsin((self.fy - self.acc_od) / g))
        roll = -1 * np.rad2deg(np.arcsin((self.fx - self.v_od * self.wz) / (g * np.cos(np.deg2rad(pitch)))))

        self.ve = self.v_od * np.sin(np.deg2rad(new_azimuth)) * np.cos(np.deg2rad(pitch))
        self.vn = self.v_od * np.cos(np.deg2rad(new_azimuth)) * np.cos(np.deg2rad(pitch))

        # self.ve = -self.v_od * np.cos(np.deg2rad(new_azimuth)) * np.cos(np.deg2rad(pitch))
        # self.vn = self.v_od * np.sin(np.deg2rad(new_azimuth)) * np.cos(np.deg2rad(pitch))
        self.vu = self.v_od  # * np.sin(np.deg2rad(pitch))

        self.delta_x = self.vn * delta_time
        self.delta_y = self.ve * delta_time
        self.delta_u = self.vu * delta_time
        self.delta_azi = delta_azimuth
        self.azimuth = new_azimuth

        #  here vn is vx, ve is vy
        # l at , long , h, ve, vn, vu, azi, a_od, w_z [states] (pitch & roll) ?
        return self.vn, self.ve, self.height, self.acc_od, pitch, roll, self.delta_x, self.delta_y, self.delta_azi

    def correct_values(self, correct_x, correct_y, correct_azi):
        self.x_pos = correct_x
        self.y_pos = correct_y
        self.azimuth = correct_azi
