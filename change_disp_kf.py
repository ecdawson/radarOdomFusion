# Kalman filter

# when fusing: tune Q matrix, something for R matrix, change correlation values, calculate timestep
import numpy as np


class ChangeDispKF:
    def __init__(self, h, delta_azi, delta_t, initial_guess, omega, v_odometer, a_odometer=0, pitch=0):
        delta_azi = np.rad2deg(delta_azi)
        pitch = np.rad2deg(pitch)
        #  state matrix = [x,y,h,v_x,v_y,vu,A,acceleration,w]
        self.phi = np.zeros((9, 9))  # np.zeros((9, 9))
        self.x = initial_guess  # 0.000001 * np.ones((9, 1))   ##initial estimate
        # self.x[6][0] = self.x[6][0]*100
        self.G = 0.001 * np.eye(9)

        self.H = np.zeros((5, 9))
        # change for only update vx vy
        # self.H = np.zeros((2, 9))

        self.Q = 0.1 * np.eye(9)  ##TUNE THIS
        # self.Q[6,6] = 0.00001

        #  covariance for noise of measurement model. want this to be dynamic eventually
        self.R = 0.1 * np.eye(5)  ##CHANGE THIS

        #  OR covariance for updating only vx vy.
        # self.R = 1 *np.eye(2)

        # FIGURE OUT CORRELATION VALUES
        self.P = 100 * np.eye(9)
        self.corr_t_gyro = 3 * 3600  # 2*2000#3*3600 # seconds
        self.std_gyro = 0.5  # degree/sec
        self.corr_t_aodo = 0.1 * 3600  # seconds
        self.std_aodo = 1  # meters/sec^2
        self.G[7, 7] = np.sqrt(2 / (self.std_gyro * self.corr_t_gyro))
        self.G[8, 8] = np.sqrt(2 / (self.std_aodo * self.corr_t_aodo))
        # self.R_m = 6378137.0
        # self.R_n = 6356752.3
        # self.g = 9.805209209982110

        # this section edited to include pitch and azimuth terms
        # CHECK IF VALUES ARE IN DEG OR RAD!!!!!
        self.phi[0, 3] = delta_t
        self.phi[1, 4] = delta_t
        self.phi[2, 5] = delta_t
        self.phi[3, 6] = delta_t * (a_odometer * np.cos(delta_azi) - v_odometer * np.sin(delta_azi) * omega) #vx
        self.phi[3, 7] = delta_t * v_odometer * np.cos(delta_azi)
        self.phi[3, 8] = delta_t * np.sin(delta_azi)
        self.phi[4, 6] = -delta_t * (a_odometer * np.sin(delta_azi) - v_odometer * np.cos(delta_azi) * omega)
        self.phi[4, 7] = delta_t * (-v_odometer * np.sin(delta_azi))
        self.phi[4, 8] = delta_t * np.cos(delta_azi)
        self.phi[5, 8] = delta_t * np.sin(np.deg2rad(pitch))
        self.phi[6, 7] = delta_t
        self.phi[7, 7] = -delta_t / self.corr_t_gyro
        self.phi[8, 8] = -delta_t / self.corr_t_aodo

        # update x,y,vx,vy,A
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 3] = 1
        self.H[3, 4] = 1
        self.H[4, 6] = 1

        # UPDATING ONLY VX VY
        # self.H[0,3] = 1
        # self.H[1,4] = 1

    def update(self, Z, delta_azi, a_odometer, delta_t, pitch, v_odometer, omega):
        pitch = np.rad2deg(pitch)
        delta_azi = np.deg2rad(delta_azi)
        # both steps with condition in the middle
        #       10*
        #       10*
        #       10*
        #       10*

        # something with the confidence
        # self.R = (0.001/(vo_points_cnt)) * np.eye(4) # feb_25/2 -> (vo_points_cnt)
        # self.R[3,3] = self.R[3,3]/10

        # fill these update steps in
        # 0.1 is the timestep
        # self.phi[3,6] = 0.1*a_odometer #* np.cos(np.deg2rad(azi)) CHECK THESE, what are they for
        # self.phi[3,8] = 0.1#*np.sin(np.deg2rad(azi))
        # self.phi[4,6] = 0.1*a_odometer #* np.sin(np.deg2rad(azi))
        # self.phi[4,8] = 0.1#*np.cos(np.deg2rad(azi))

        self.phi[0, 3] = delta_t
        self.phi[1, 4] = delta_t
        self.phi[2, 5] = delta_t
        self.phi[3, 6] = delta_t * (a_odometer * np.cos(delta_azi) - v_odometer * np.sin(delta_azi) * omega) #vx
        self.phi[3, 7] = delta_t * v_odometer * np.cos(delta_azi)
        self.phi[3, 8] = delta_t * np.sin(delta_azi)
        self.phi[4, 6] = -delta_t * (a_odometer * np.sin(delta_azi) - v_odometer * np.cos(delta_azi) * omega)
        self.phi[4, 7] = delta_t * (-v_odometer * np.sin(delta_azi))
        self.phi[4, 8] = delta_t * np.cos(delta_azi)
        self.phi[5, 8] = delta_t * np.sin(np.deg2rad(pitch))
        self.phi[6, 7] = delta_t
        self.phi[7, 7] = -delta_t / self.corr_t_gyro
        self.phi[8, 8] = -delta_t / self.corr_t_aodo

        # "measurement matrix
        # z = np.matrix(Z).T
        z = Z

        # 'a priori estimate'
        X_k_pri = np.dot((self.phi), (self.x))
        P_k_pri = np.add(np.dot((self.phi), np.dot((self.P), (self.phi.T))),
                         np.dot((self.G), np.dot((self.Q), (self.G.T))))

        # 'this is specific to visual odometry (?) adjusting R based on point count. we can adjust R based on how many object detecitions?'
        # use factors that affect the radar -> make some kind of metric, turn reliability factors into a number (weighting factor)
        # example: number of objects
        # self.R = (0.001/(vo_points_cnt)) * np.eye(4) # feb_25/2 -> (vo_points_cnt)
        # self.R[3,3] = self.R[3,3]/10

        # 'calculate K based on R and a priori estimate'
        Kg = np.dot(np.dot((P_k_pri), (self.H.T)),
                    np.linalg.inv(np.add(np.dot((self.H), np.dot((P_k_pri), (self.H.T))), (self.R))))

        # 'a posteriori estimate'
        self.x = np.add((X_k_pri), np.dot((Kg), np.subtract((z), np.dot((self.H), (X_k_pri)))))  # Z sholud be in data
        self.P = np.dot(np.eye(9) - np.dot((Kg), (self.H)), (P_k_pri))  # P_k_post
        self.P = 0.5 * np.add(self.P, self.P.T)

        # 'return x state variables of interest. we need to change these as we have different updates'
        # returning x,y,vx,vy,Azimuth,P
        return self.x, self.P

    def tune_R(self, direction_flag, num_targets, num_inliers):
        # if direction_flag == 1:
        # going forwards - radar is reliable
        # coef = -10
        # else:
        coef = 0.0001
        self.R = coef * np.eye(5)

