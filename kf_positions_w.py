# Kalman filter

# when fusing: tune Q matrix, something for R matrix, change correlation values, calculate timestep
import numpy as np

class kf_meter:
    def __init__(self, h, azi, delta_t, initial_guess, a_odometer=0, pitch=0):
        self.threshold = 15 # Threshold for outage
        self.upper_threshold = 25
        self.outage_duration_thresh = 3
        self.outage_times = []
        self.sensor_confidence = []
        self.outage = 0
        self.outage_start_time = 0

        azi = np.rad2deg(azi)
        pitch = np.rad2deg(pitch)
        #  state matrix = [x,y,h,v_x,v_y,vu,A,acceleration,w]
        self.phi = np.zeros((9, 9))  # np.zeros((9, 9))
        self.x = initial_guess  # 0.000001 * np.ones((9, 1))   ##initial estimate
        # self.x[6][0] = self.x[6][0]*100
        self.G = 0.001 * np.eye(9)

        # self.H = np.zeros((6, 9))
        self.H = np.zeros((5, 9))
        # change for only update vx vy
        # self.H = np.zeros((2, 9))

        # NO X Y IN MEASUREMENT
        # self.H = np.zeros((3, 9))

        self.Q = 1 * np.eye(9)  ##TUNE THIS
        self.Q[8, 8] = 0.001

        #  covariance for noise of measurement model. want this to be dynamic eventually
        # self.R = 0.1 * np.eye(6)  ##CHANGE THIS
        self.R = 0.1 * np.eye(5)
        #  OR covariance for updating only vx vy.
        # self.R = 1 *np.eye(2)

        # NO X Y IN MEASUREMENT
        # self.R = 0.1 * np.eye(3)

        # FIGURE OUT CORRELATION VALUES
        self.P = 5 * np.eye(9)
        self.corr_t_gyro = 3 * 3600  # 2*2000#3*3600 # seconds
        self.std_gyro = 0.5  # degree/sec
        self.corr_t_aodo = 0.1 * 3600  # seconds
        self.std_aodo = 1  # meters/sec^2
        self.G[8, 8] = np.sqrt(2 / (self.std_gyro * self.corr_t_gyro))
        self.G[7, 7] = np.sqrt(2 / (self.std_aodo * self.corr_t_aodo))
        # self.R_m = 6378137.0
        # self.R_n = 6356752.3
        # self.g = 9.805209209982110

        # this section edited to include pitch and azimuth terms
        # CHECK IF VALUES ARE IN DEG OR RAD!!!!!
        #
        # self.phi[0, 4] = delta_t
        # self.phi[1, 3] = delta_t

        self.phi[0, 3] = delta_t
        self.phi[1, 4] = delta_t
        self.phi[2, 5] = delta_t
        self.phi[3, 6] = delta_t * a_odometer * np.cos(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[3, 7] = delta_t * np.sin(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[4, 6] = -delta_t * a_odometer * np.sin(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[4, 7] = delta_t * np.cos(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[5, 7] = delta_t * np.sin(np.deg2rad(pitch))
        self.phi[6, 8] = delta_t
        self.phi[8, 8] = -delta_t / self.corr_t_gyro
        self.phi[7, 7] = -delta_t / self.corr_t_aodo

        # update x,y,vx,vy,w,A
        self.H[0, 0] = 1 # x
        self.H[1, 1] = 1 # y
        self.H[2, 3] = 1 # vx
        self.H[3, 4] = 1 # vy
        # self.H[4, 8] = 1 # w
        # self.H[5, 6] = 1 # A

        self.H[4, 6] = 1 # A

        # # NO X Y IN MEASUREMENT
        # self.H[0, 3] = 1 # vx
        # self.H[1, 4] = 1 # vy
        # self.H[2, 8] = 1 # w

        # UPDATING ONLY VX VY
        # self.H[0,3] = 1
        # self.H[1,4] = 1

    def update(self, Z, azi, a_odometer, delta_t, pitch, num_inliers1, num_inliers2):
        azi = np.rad2deg(azi)
        pitch = np.rad2deg(pitch)

        if self.outage:
            return np.zeros(9), np.zeros((9, 9))

        # self.phi[0, 4] = delta_t
        # self.phi[1, 3] = delta_t

        self.phi[0, 3] = delta_t
        self.phi[1, 4] = delta_t
        self.phi[2, 5] = delta_t
        self.phi[3, 6] = delta_t * a_odometer * np.cos(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[3, 7] = delta_t * np.sin(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[4, 6] = -delta_t * a_odometer * np.sin(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[4, 7] = delta_t * np.cos(np.deg2rad(azi)) * np.cos(np.deg2rad(pitch))
        self.phi[5, 7] = delta_t * np.sin(np.deg2rad(pitch))
        self.phi[6, 8] = delta_t
        self.phi[8, 8] = -delta_t / self.corr_t_gyro
        self.phi[7, 7] = -delta_t / self.corr_t_aodo

        # measurement matrix
        # z = np.matrix(Z).T, [x, y, vx, vy, a]
        z = Z

        # a priori estimate
        X_k_pri = np.dot(self.phi, self.x)
        P_k_pri = np.add(np.dot(self.phi, np.dot(self.P, self.phi.T)),
                         np.dot(self.G, np.dot(self.Q, self.G.T)))

        # calculate K based on R and a priori estimate

        Kg = np.dot(np.dot(P_k_pri, self.H.T),
                    np.linalg.inv(np.add(np.dot(self.H, np.dot(P_k_pri, self.H.T)), self.R)))

        # 'a posteriori estimate'
        self.x = np.add(X_k_pri, np.dot((Kg), np.subtract((z), np.dot(self.H, X_k_pri))))  # Z sholud be in data
        self.P = np.dot(np.eye(9) - np.dot((Kg), self.H), P_k_pri)  # P_k_post
        self.P = 0.5 * np.add(self.P, self.P.T)

        # 'return x state variables of interest. we need to change these as we have different updates'
        # returning x,y,vx,vy,Azimuth,P
        # return np.zeros(9), np.zeros((9, 9))  # to see solution with no corrections
        return self.x, self.P

    def tune_vel(self, num_inliers1, num_inliers2):
        # output1 = self.MapOutput(num_inliers1)
        # output2 = self.MapOutput(num_inliers2)
        output1 = self.MapExponential(num_inliers1, 15)
        output2 = self.MapExponential(num_inliers2, 15)
        # print(num_inliers1)
        # print(num_inliers2)
        # print(output2)
        coef = np.maximum(output1, output2)
        if coef in [100, 10, 1]:
            self.sensor_confidence.append(0)
        else:
            self.sensor_confidence.append(1)
        return 0.00001# coef

    def tune_omega(self, num_inliers1, num_inliers2):
        # if num_inliers1 > 20 or num_inliers2 > 20:
        #     coef = 0.1
        # # else:
        # else:
        #     coef = 1000
        output1 = self.MapExponential(num_inliers1, 15)
        output2 = self.MapExponential(num_inliers2, 15)
        coef = np.maximum(output1, output2)
        return coef
        # output1 = self.MapOutput(num_inliers1)
        # output2 = self.MapOutput(num_inliers2)
        # coef = np.maximum(output1, output2)
        # return coef

    def tune_R(self, direction_flag, num_inliers1, num_inliers2):
        # if direction_flag == 1:
        # going forwards - radar is reliable
        v_coef = self.tune_vel(num_inliers1, num_inliers2)
        w_coef = self.tune_omega(num_inliers1, num_inliers2)
        self.R[0, 0] = 10*v_coef  # x
        self.R[1, 1] = 10*v_coef  # y
        self.R[2, 2] = v_coef #vx
        self.R[3, 3] = v_coef #vy
        self.R[4, 4] = 500# w_coef# azimuth
        # self.R[5, 5] = w_coef  # omega

        # # NO X Y IN MEASUREMENT
        # self.R[0, 0] = v_coef # v_coef  # vx
        # self.R[1, 1] = v_coef # v_coef  # vy
        # self.R[2, 2] = w_coef #vx       # omega


    def MapOutput(self, detection):
        # print(detection)
        minimumDetections = 5
        maximumDetections = 45
        Step = 5
        intial = np.arange(minimumDetections, maximumDetections, Step)
        output = np.asarray([100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0])
        lowerBoundary = detection < intial[0]
        higherBoundary = detection > intial[-1]
        missingValue = np.isnan(detection)
        if lowerBoundary:
            return output[0]
        elif higherBoundary or missingValue:
            return output[-1]
        for i in range(len(intial)-1):
            InRange = (detection >= intial[i]) & (detection <= intial[i+1])
            if InRange:
                return output[i]

    def MapExponential(self, detection, threshold, constant = 1):

        if np.isnan(detection):
            coef = 0
        else:
            coef = np.exp(constant * (15-detection))
        return coef

    def get_confidence_list(self):
        return self.sensor_confidence

    def handle_outage(self, num_inliers1, num_inliers2, time):
        outage_start = 0
        outage_end = 0
        if not self.outage:
            if num_inliers1 < self.threshold and num_inliers2 < self.threshold:
                self.outage = 1
                outage_start = 1
        if self.outage:
            if num_inliers1 > self.upper_threshold and num_inliers2 > self.upper_threshold:
                self.outage = 0
                outage_end = 1
        self.outage_times.append((time, self.outage))
        return outage_start, outage_end

    def handle_outage_with_duration_constraint(self, num_inliers1, num_inliers2, time):
        outage_start = 0
        outage_end = 0
        if not self.outage:
            if num_inliers1 < self.threshold and num_inliers2 < self.threshold:
                self.outage = 1
                outage_start = 1
                self.outage_start_time = time
        else:
            outage_duration = (time - self.outage_start_time)/1000
            if num_inliers1 > self.upper_threshold and num_inliers2 > self.upper_threshold:
                if outage_duration >= self.outage_duration_thresh:
                    self.outage = 0
                    outage_end = 1
        self.outage_times.append((time, self.outage))
        return outage_start, outage_end

    def get_outages(self):
        return self.outage_times

    def is_outage(self):
        if self.outage:
            return 1
        else:
            return 0
