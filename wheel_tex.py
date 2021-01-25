# wheel tex class
import numpy as np


class wheel_sensor:

    def __init__(self, azimuth, x_pos, y_pos):
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.azimuth = azimuth
        self.x_pos = x_pos
        self.y_pos = y_pos

    def update(self, delta_time, speed, yaw_rate):
        self.speed = speed
        self.yaw_rate = yaw_rate

        delta_azimuth = self.yaw_rate * delta_time
        new_azimuth = self.azimuth + delta_azimuth

        if new_azimuth >= 360:
            new_azimuth = new_azimuth - 360.0
        elif new_azimuth < 0:
            new_azimuth = new_azimuth + 360.0

        vx = self.speed * np.cos(np.deg2rad(new_azimuth))
        vy = self.speed * np.sin(np.deg2rad(new_azimuth))

        self.x_pos = self.x_pos + vx * delta_time
        self.y_pos = self.y_pos + vy * delta_time

        self.azimuth = new_azimuth

        return self.x_pos, self.y_pos, self.azimuth, vx, vy

    def correct_values(self, correct_x, correct_y, correct_azi):
        self.x_pos = correct_x
        self.y_pos = correct_y
        self.azimuth = correct_azi
