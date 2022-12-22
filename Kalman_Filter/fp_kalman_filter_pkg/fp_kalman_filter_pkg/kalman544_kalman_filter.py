#!/usr/bin/env python3

import numpy as np
from numpy.random import randn
from math import atan2, asin, sqrt
import matplotlib.pyplot as plt

import json

# importing the UnivariateSpline interpolation to help make data set sizes match
# a zero order hold would have been a better way to go about doing this
# but should have been done during the data collection step
from scipy.interpolate import UnivariateSpline

# library to help calculated MSE
from sklearn.metrics import mean_squared_error

## Example code written by: Christian Mele, adapted to Python by Yue Hu

class KalmanFilter():
    def __init__(self):

        # import tf data from json file
        # Opening JSON file
        json_file = open('bag_file_data.json')
        # returns JSON object as 
        # a dictionary
        self.bag_data = json.load(json_file)
        json_file.close()

        # number of states in the state vector
        # x, x_dot, theta, omega
        self.num_states = 4
        self.imu_data = self.bag_data['imu']['imu_data']
        self.encoder_data = self.bag_data['encoder']


        # Determine time step based on sensor publishing rates:
        #  In ROS we get the following:

        # ros2 topic hz /joint_states

        # average rate: 28.953
	    # min: 0.030s max: 0.040s std dev: 0.00158s window: 615

        # ros2 topic hz /imu

        # average rate: 196.652
	    # min: 0.001s max: 0.091s std dev: 0.00157s window: 4735

        # The Kalman Filter will be run at the slowest rate of the two 
        # publishers at 0.04 s
    
        # Time step of analysis
        self.dt = 0.04

        ## Prediction estimation - this is your "Priori"
        # we will be using IMU values for the project, however in this example we
        # use a block with a spring attached to it
        # self.xhat = np.matrix([0.5, 1, 0]).transpose() # mean(mu) estimate for the "first" step

        # assume initial states all at 0 
        self.xhat = np.matrix([0, 0, 0, 0]).transpose()


        self.P = np.identity(self.num_states) # covariance initial estimation between the
        # the covariance matrix P is the weighted covariance between the
        # motion model definition - establish your robots "movement"

        # self.A = np.matrix([[1, self.dt, 1 / 2 * self.dt ** 2], [0, 1, self.dt], [k / m, 0, 0]]) # this is the state space

        #  state transition matrix
        # x, x_dot theta, omega
        self.A = np.matrix([[1, self.dt, 0, 0], 
                            [0, 1, 0, 0], 
                            # [0, 0, 1, 0, 0], 
                            [0, 0, 1, self.dt],
                            [0, 0, 0, 1]  
                            ]) # this is the state space

        # input a_x, angular acceleration alpha
        # self.B = np.matrix([0, 0, 1/m]).transpose() # this is the "input"

        # Control matrix B for an input of linear acceleration and angular acceleration
        self.B = np.matrix(([0.5*self.dt**2, self.dt, 0, 0], 
                            [0, 0, 0.5*self.dt**2, self.dt])).transpose() # this is the "input"
        # self.B = np.matrix([0, 0, 1]).transpose() # we can also treat the input as the "acceleration" for that step as calculated by an IMU!
        # self.Q = np.identity(3)*0.05 # this is the model variance/covariance (usually we treat it as the input matrix squared).
                                     # these are the discrete components, found by performing a taylor's
                                     # expansion on the continuous model
        # self.Q = np.identity(3)*0.05

        cov_a_x = self.imu_data[0]['linear_acceleration_covariance'][0]
        cov_a_y = self.imu_data[0]['linear_acceleration_covariance'][0]
        cov_a = np.sqrt(cov_a_x**2 + cov_a_y**2)
        cov_omega = self.imu_data[0]["angular_velocity_covariance"][0]

        # setup process noise matrix Q
        # PROJECTION USING THE STATE TRANSITION MATRIX
        # Q_a = np.zeros((4,4))
        # Q_a[1,1] = cov_a
        # Q_a[3,3] = cov_omega
        # cov_matrix = np.array([cov_a, cov_omega]).transpose()

        Q_a = np.array([
            [1/4*self.dt**4*cov_a, 1/2*self.dt**3*cov_a, 0, 0],
            [1/2*self.dt**3*cov_a, self.dt**2*cov_a, 0, 0],
            [0, 0, 1/4*self.dt**4*cov_omega, 1/2*self.dt**3*cov_omega],
            [0, 0, 1/2*self.dt**3*cov_omega, self.dt**2*cov_omega],
        ])

        self.Q = self.A*Q_a*self.A.transpose()

        ## Measurement Model - Update component
        # self.C = np.matrix([[-1, 0, 0], [0, 0.6, 0]]) # what this represents is our "two" sensors, both with linear relationships
                                                      # to position and velocity respectively

        # in actual environments, what this does is translate our measurement to a
        # voltage or some other metric that can be ripped directly from the sensor
        # when taking online measurements. We compare those values as our "error"

        # self.R = np.matrix([[0.000005, 0], [0, 0.000005]]) # this is the sensor model variance-usually characterized to accompany
                                                  # the sensor model already starting

        # R covariance values determined by trial and error looking at the comaparison plots and mean square values
        self.R = np.identity(self.num_states)
        self.R[0,0] = 1e-09
        # 1e-05
        self.R[1,1] = 1e-05
        self.R[2,2] = 1e9
        self.R[3,3] = 1e9

        # self.R = np.matrix([[cov_a_x, 0], [0, cov_omega]])
        # self.R = np.identity(self.num_states)*0.0001

        # turtlebot 3 wheel radius data obtained from https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications
        self.wheel_radius = (66/2)*10**-3 # 66 mm

        self.robo_width = (self.bag_data['tf']['left_wheel_to_base'][0]['translation']['y']) - (self.bag_data['tf']['right_wheel_to_base'][0]['translation']['y'])

        # observation matrix C if directly passing wheel speeds from encoders

        # self.C = np.array([
        #     [self.dt/2, 1/2, self.dt/self.robo_width, 1/self.robo_width],
        #     [self.dt/2, 1/2, self.dt/self.robo_width, 1/self.robo_width],
        # ])*self.wheel_radius

        # observation matrix c if handling wheels speed inputs somewhere else
        self.C = np.identity(self.num_states)

        # Initialize Ground Truth Data obtained from the tf topic
        self.tf_ground_truth = {'x': [], 'theta' : []}

    # helper function to convert rad to m
    def rad_to_m(self, rad_dist):
    
        return rad_dist* self.wheel_radius

    # helper function assumes the robot's position is the average of its wheels
    def pose_from_2_encoders(self, left_encoder_dist, right_encoder_dist):
        return (left_encoder_dist + right_encoder_dist)/2

    # helper function to convert a quaternion to euler form
    # credit to https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return {'roll_x': roll_x, 'pitch_y': pitch_y, 'yaw_z': yaw_z} # in radians
    

    # helper function to calculate the mean square error
    def helper_mean_square_error(self, ground_truth, kalman_filter):
        return np.mean(np.square(kalman_filter - ground_truth))

    def set_tf_data(self):
        # print(self.tf_data['tf'])
        tf_bf_to_odom = self.bag_data['tf']["base_footprint_to_odom"]
        num_data_point = len(tf_bf_to_odom)
        ground_truth_pose = np.zeros((num_data_point, 3))
        tf_rotation = np.zeros((num_data_point, 1))

        # # fill the empty ground truth array
        for index, pose in enumerate(tf_bf_to_odom):
            ground_truth_pose[index][0] = pose['translation']['x']
            ground_truth_pose[index][1] = pose['translation']['y']
            ground_truth_pose[index][2] = pose['translation']['z']

            tf_rotation[index][0] = self.euler_from_quaternion(pose['rotaton']['x'], 
            pose['rotaton']['y'], pose['rotaton']['z'], pose['rotaton']['w'])['yaw_z']


        # set the ground truth data
        self.tf_ground_truth['x'] = ground_truth_pose[:, 0]
        self.tf_ground_truth['theta'] = tf_rotation[:, 0]

        # some plotting code for debugging

        # print(ground_truth_pose)

        # T = np.arange(0, 26, self.dt)
        # plt.figure()
        # T = np.arange(0, 26, 26/756)
        # # plt.plot(ground_truth_pose[:, 0], ground_truth_pose[:, 1])
        # plt.plot(T, ground_truth_pose[:, 0])
        # plt.xlabel("X distance [m]")
        # plt.ylabel("Y distance [m]")
        # plt.title("Ground truth X vs Y distance of Path from /tf topic")
        # plt.grid(visible=True)

        # plt.show()

        # plt.figure()
        # T = np.arange(0, 26, 26/756)
        # plt.plot(T, tf_rotation[:, 0])
        # # plt.plot(T, ground_truth_pose[:, 0])
        # plt.xlabel("Time")
        # plt.ylabel("Angle")
        # plt.title("Ground truth Angle vs Y distance of Path from /tf topic")
        # plt.grid(visible=True)

        # plt.show()


        # print(ground_truth_pose[:, 0])

    # plotting code for debugging
    
    def plot_sensor_data(self):

        encoder_position = []
        encoder_velocity = []

        # print("Encoder final velo", self.encoder_data['wheel_left_joint']['velocity'][-1]*26)

        for dist in self.encoder_data['wheel_right_joint']['position']:
            encoder_position.append(dist/32)

        plt.figure()
        T = np.arange(0, len(self.encoder_data['wheel_left_joint']['position']), )
        # plt.plot(ground_truth_pose[:, 0], ground_truth_pose[:, 1])
        plt.plot(T, encoder_position)
        plt.xlabel("T")
        plt.ylabel("Encoder distance [m]")
        # plt.title("Ground truth X vs Y distance of Path from /tf topic")
        # plt.grid(visible=True)

        plt.show()

        return
    
    # plotting code for debugging

    def plot_imu_data(self):

        rotation = []
        angular_velocity = []
        linear_acceleration = []
        index_array = []
        vel_x = []
        dist_x = []

        for index, data in enumerate(self.imu_data):

            rotation.append(self.euler_from_quaternion(data['rotation']['x'], data['rotation']['y'], data['rotation']['w'], data['rotation']['w'])['yaw_z'])
            angular_velocity.append(data['angular_velocity']['x'])
            linear_acceleration.append(data['linear_acceleration']['x'])
            index_array.append(index*2)

            if index == 0:
                vel_x.append(0 + linear_acceleration[index])
                dist_x.append(0 + 0.5*linear_acceleration[index]**2)
            else:
                vel_x.append(vel_x[index-1] + linear_acceleration[index])
                dist_x.append(dist_x[index-1] + 0.5*linear_acceleration[index]**2)

        plt.figure()
        # T = np.arange(0, len(self.encoder_data['wheel_left_joint']['position']), )
        # plt.plot(ground_truth_pose[:, 0], ground_truth_pose[:, 1])
        # plt.plot(index_array, rotation)
        # plt.plot(index_array, angular_velocity)
        # plt.plot(index_array, linear_acceleration)
        plt.plot(index_array, vel_x)
        plt.plot(index_array, dist_x)
        # plt.xlabel("T")
        # plt.ylabel("Encoder distance [m]")
        # plt.title("Ground truth X vs Y distance of Path from /tf topic")
        plt.legend(['ang v','linear a', 'vel', 'dist'])
        plt.grid(visible=True)

        plt.show()

        return
    
    # helper function, takes an array and increasses its size
    # linearly interpolating between values
    def interpolate_array(self, old_array, new_length):

        old_indices = np.arange(0,len(old_array))
        new_indices = np.linspace(0,len(old_array)-1,new_length)
        spline = UnivariateSpline(old_indices,old_array,k=3,s=0)
        new_array = spline(new_indices)

        return new_array


    def run_kalman_filter(self, Tfinal):

        T = np.arange(0, Tfinal, self.dt)
        xhat_S = np.zeros([4, len(T) + 1])
        x_S = np.zeros([4, len(T) + 1])
        x = np.zeros([4, len(T) + 1])
        x[:, [0]] = self.xhat
        # y = np.zeros([4, len(T)])
        y_hat = np.zeros([self.num_states, len(T)])
        # y = np.zeros([2, len(T)])
        # y_hat = np.zeros([2, len(T)])

        # linearly interpolating the values in the encoder data set
        # to expand it to match the size of the time series for k

        encoder_pr = self.encoder_data['wheel_right_joint']['position']
        encoder_pl = self.encoder_data['wheel_left_joint']['position']
        encoder_ur = self.encoder_data['wheel_right_joint']['velocity']
        encoder_ul = self.encoder_data['wheel_left_joint']['velocity']

        expanded_encoder_pr = self.interpolate_array(encoder_pr, len(T))
        expanded_encoder_pl = self.interpolate_array(encoder_pl, len(T))
        expanded_encoder_ur = self.interpolate_array(encoder_ur, len(T))
        expanded_encoder_ul = self.interpolate_array(encoder_ul, len(T))


        for k in range(len(T)):
            # u = 0.01 # normally you'd initialise this above

            #  imu data
            imu_measure = self.imu_data[k]
            a_x = imu_measure['linear_acceleration']['x']
            a_y = imu_measure['linear_acceleration']['y']
            imu_omega = imu_measure['angular_velocity']['z']

            # encoder data
            # pr = self.encoder_data['wheel_right_joint']['position'][k]
            # pl = self.encoder_data['wheel_left_joint']['position'][k]
            # ur = self.encoder_data['wheel_right_joint']['velocity'][k]
            # ul = self.encoder_data['wheel_left_joint']['velocity'][k]
            pr =  expanded_encoder_pr[k]
            pl =  expanded_encoder_pl[k]
            ur =  expanded_encoder_ur[k]
            ul =  expanded_encoder_ul[k]

            # encoder data length
            # time scale length

            # print("Encoder data length", len(self.encoder_data['wheel_right_joint']['position']))
            # print("Time scale length", len(T))

            # linearly interpolate between values in encoder array to change size to match len(T)
            pr_range = self.encoder_data['wheel_right_joint']['position'][-1] - self.encoder_data['wheel_right_joint']['position'][0]
            pl_range = self.encoder_data['wheel_right_joint']['position'][-1] - self.encoder_data['wheel_left_joint']['position'][0]
            ur_range = self.encoder_data['wheel_right_joint']['velocity'][-1] - self.encoder_data['wheel_right_joint']['velocity'][0]
            ul_range = self.encoder_data['wheel_right_joint']['velocity'][-1] - self.encoder_data['wheel_left_joint']['velocity'][0]


            accel_input  = sqrt(a_x**2 + a_y**2)


            # soft_sensor_omega = []

            # for index, value in enumerate(ur):
            #     soft_sensor_omega.append((ur[index] - ul[index])/robo_width)
                # print(index)

            # soft_sensor_omega = (ur - ul)/robo_width

            # soft sensor omega

            # u = np.array([a_x, soft_sensor_omega[k]]).reshape((2, 1))

            # inputs used are the linear and angular accelerations from the imu
            u = np.array([abs(accel_input), imu_omega]).reshape((2, 1))
            # u = np.array([accel_input, imu_omega]).reshape((2, 1))

            #### Simulate motion with random motion disturbance ####
            # w = np.matrix([self.Q[0, 0] * randn(1), self.Q[1, 1] * randn(1), self.Q[2, 2] * randn(1)])

            # update state - this is a simulated motion and is PURELY for fake
            # sensing and would essentially be
            # x[:, [k + 1]] = self.A * x[:, [k]] + self.B * u + w

            # taking a measurement - simulating a sensor
            # create our sensor disturbance
            # v = np.matrix([self.R[0] * randn(1), self.R[1] * randn(1), self.R[1] * randn(1), self.R[1] * randn(1)])
            # create this simulated sensor measurement
            # y[:, [k]] = self.C*x[:, [k+1]] + v

            # read encoder data for measurements
            # before we send off the encoder values for processing let's do some quick conversions to 
            # make things a bit easier
            # assuming the position and velocity of the robot is
            # the average of both wheels
            position_from_encoders = self.rad_to_m((pr + pl)/2)
            velocity_from_encoders = self.rad_to_m((ur + ul)/2)
            # not too sure how good this theta approximation is
            # but for our purposes it might be good enough?
            theta_from_encoders = (pr-pl)/self.robo_width
            omega_from_encoders = self.wheel_radius * (ur - ul)/(self.robo_width)
            y = np.matrix([
                position_from_encoders,
                velocity_from_encoders,
                theta_from_encoders,
                omega_from_encoders
            ]).transpose()

            #########################################
            ###### Kalman Filter Estimation #########
            #########################################
            # Prediction update
            xhat_k = self.A * self.xhat + self.B * u # we do not put noise on our prediction
            P_predict = self.A*self.P*self.A.transpose() + self.Q
            # this co-variance is the prediction of essentially how the measurement and sensor model move together
            # in relation to each state and helps scale our kalman gain by giving
            # the ratio. By Definition, P is the variance of the state space, and
            # by applying it to the motion model we're getting a motion uncertainty
            # which can be propogated and applied to the measurement model and
            # expand its uncertainty as well

            # Measurement Update and Kalman Gain
            K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)
            # the pseudo inverse of the measurement model, as it relates to the model covariance
            # if we don't have a measurement for velocity, the P-matrix tells the
            # measurement model how the two should move together (and is normalised
            # in the process with added noise), which is how the kalman gain is
            # created --> detailing "how" the error should be scaled based on the
            # covariance. If you expand P_predict out, it's clearly the
            # relationship and cross-projected relationships, of the states from a
            # measurement and motion model perspective, with a moving scalar to
            # help drive that relationship towards zero (P should stabilise).

            # self.xhat = xhat_k + K * (y[:, [k]] - self.C * xhat_k)
            self.xhat = xhat_k + K * (y - self.C * xhat_k)

            self.P = (np.identity(self.num_states) - K * self.C) * P_predict # the full derivation for this is kind of complex relying on
                                             # some pretty cool probability knowledge

            # Store estimate
            xhat_S[:, [k]] = xhat_k
            x_S[:, [k]] = self.xhat
            y_hat[:, [k]] = self.C*self.xhat

        return x, xhat_S, x_S, y_hat

    def plot_results(self, Tfinal, x, xhat_S, x_S):
        T = np.arange(0, Tfinal, self.dt)
        plt.figure()
        plt.grid(True)
        plt.plot(T, x_S[0,0:-1])
        plt.plot(T, x_S[1,0:-1])
        plt.plot(T, x_S[2,0:-1])
        plt.plot(T, x_S[3,0:-1])

        # plt.plot(T, x[0,1:])
        # plt.plot(T, x[1,1:])
        # plt.plot(T, x[2,1:])
        plt.legend(['position est.','vel estimate', 'theta est', 'omega est'])
        plt.grid(True)
        # plt.legend(['position est.','vel estimate', 'accel est', 'true pos', 'true vel', 'true accel'])

        # print("X", x[0,1:])

        # tf_bf_to_odom = self.bag_data['tf']["base_footprint_to_odom"]
        # num_data_point = len(tf_bf_to_odom)
        # ground_truth_pose = np.zeros((num_data_point, 3))

        # # print(num_data_point)
        # # print(ground_truth_pose[0][0])
        # # print(tf_bf_to_odom[0]['translation']['x'])
        # # # fill the empty ground truth array
        # for index, pose in enumerate(tf_bf_to_odom):
        #     ground_truth_pose[index][0] = pose['translation']['x']
        #     ground_truth_pose[index][1] = pose['translation']['y']
        #     ground_truth_pose[index][2] = pose['translation']['z']

        # # print(ground_truth_pose)

        # # T = np.arange(0, 26, self.dt)
        # # plt.figure()

        # plt.plot(ground_truth_pose[:, 0], ground_truth_pose[:, 1])
        # # plt.plot(T, ground_truth_pose[:, 0])
        # # plt.xlabel("X distance [m]")
        # # plt.ylabel("Y distance [m]")
        # # plt.title("Ground truth X vs Y distance of Path from /tf topic")
        # plt.grid(visible=True)

        plt.show()

        plt.figure()
        plt.plot(T, xhat_S[0,0:-1])
        plt.plot(T, xhat_S[1,0:-1])
        plt.plot(T, xhat_S[2,0:-1])
        plt.plot(T, xhat_S[3,0:-1])
        # plt.plot(T, x[0,1:])
        # plt.plot(T, x[1,1:])
        # plt.plot(T, x[2,1:])
        # plt.legend(['position pred.','vel pred.', 'theta pred.', 'true pos', 'true vel', 'true accel'])
        plt.legend(['position pred.','vel pred.', 'theta pred.', 'omega pred.'])
        plt.grid(True)

        # print(x)

        # adjust predictions to be same size as ground truth by linearly interpolating between the 
        # values

        x_update_interpolated = self.interpolate_array(x_S[0,0:-1], len(self.tf_ground_truth['x']))
        theta_update_interpolated = self.interpolate_array(x_S[2,0:-1], len(self.tf_ground_truth['theta']))

        tf_dist_interpolated_to_time = self.interpolate_array(self.tf_ground_truth['x'], len(T))
        tf_theta_interpolated_to_time = self.interpolate_array(self.tf_ground_truth['theta'], len(T))

        plt.figure()
        plt.scatter(T, tf_dist_interpolated_to_time)
        plt.scatter(T, x_S[0,0:-1])
        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')

        plt.figlegend(['TF True Position', 'Estimated Position'])
        plt.grid(True)

        plt.figure()
        plt.scatter(T, tf_theta_interpolated_to_time)
        plt.scatter(T, x_S[2,0:-1])
        plt.xlabel('Time [s]')
        plt.ylabel('Yaw [rad]')
        plt.figlegend(['TF True Theta', 'Estimated Theta'])

        plt.grid(True)

        plt.show()

        print("Mean Squared Error (MSE)")
        x_mse = self.helper_mean_square_error(tf_dist_interpolated_to_time,  x_S[0,0:-1])
        theta_mse = self.helper_mean_square_error(tf_theta_interpolated_to_time, x_S[2,0:-1])
        print("X:", x_mse)
        print("Theta:", theta_mse)

def main():
    print("MTE544 Final Project - Kalman Filter")
    kf = KalmanFilter()
    Tfinal = 26

    kf.set_tf_data()
    x, xhat_S, x_S, y_hat = kf.run_kalman_filter(Tfinal)
    kf.plot_results(Tfinal, x, xhat_S, x_S)

if __name__ == '__main__':
    main()