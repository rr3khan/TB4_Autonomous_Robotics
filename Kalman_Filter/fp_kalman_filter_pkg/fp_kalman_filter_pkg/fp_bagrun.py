# Code adapted from the TA Christian's Lab 2 bagreader package
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from tf2_msgs.msg import TFMessage
# Import data types needed for sensor data and global truth
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from turtlesim.msg import Pose
# imports for frame transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Transform, Vector3, Quaternion
#
from math import atan2, asin

# import pandas as pd could not get pandas to work in the VM running into installation errors
# import numpy as np
# using json for the data export instead
import json



class Bagreader(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('bagreader')
        # create the subscriber object
        # tf call back
        # self.tf_sub = self.create_subscription(
        #     LaserScan, '/tf', self.tf_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.tf_sub = self.create_subscription(
        TFMessage, '/tf', self.tf_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.tf_static_sub = self.create_subscription(
        TFMessage, '/tf_static', self.tf_static_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # imu call back
        # self.timer_pose = self.create_timer(0.5, self.update_current_pose)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # encoder call back
        self.encoder_sub = self.create_subscription(
            JointState, '/joint_states', self.encoder_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # define the timer period for 0.5 seconds
        # self.timer_period = 0.5
        # define the variable to hold the received laser data
        self.laser_forward = 0
        # define the variable to hold the received imu data
        self.imu_data = Imu()
        # define the variable to hold the received tf data
        self.tf_data = TFMessage()
        self.tf_static_data = TFMessage()
        # define the variable to hold the received encoder data
        self.encoder_data = JointState()
        # Credit to tf  listener tutorial https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # initialize json format to hold the data we want from the bag file
        self.path_data = {
            'tf': {
                'base_footprint_to_odom': [],
                'left_wheel_to_base': [],
                'right_wheel_to_base': []
            },
            'tf_static': {
                'base_link_to_base_footprint': [],
                'imu_link_to_base_link': []

            },
            'imu': {
                'frame': '',
                'imu_data': []

            },
            'encoder': {
                'wheel_left_joint' : {
                    'position': [],
                    'velocity': []

                }, 
                'wheel_right_joint' : {
                    'position': [],
                    'velocity': []

                }
            }
        }


    # need to setup the laser callback message

    def laser_callback(self, msg):
        self.laser_forward = msg
        print(self.laser_forward)
        # this takes the value at angle 359 (equivalent to angle 0)

    # helper function to get pose translation and rotation data from a transform message
    def extract_pose_data(self, transform_data: Transform):
        return {
            'translation': {'x' : transform_data.translation.x, 'y' : transform_data.translation.y, 'z' : transform_data.translation.z},
            'rotaton': {'x' : transform_data.rotation.x, 'y' : transform_data.rotation.y, 'z' : transform_data.rotation.z, 'w' : transform_data.rotation.w}
        }
    
    def export_to_json(self):

        # Convert Data to a JSON string
        json_data = json.dumps(self.path_data)

        # # Export the JSON string to a file
        with open("bag_file_data.json", "w") as json_file:
            json_file.write(json_data)
        

    # need to setup the tf callback message

    def tf_callback(self, msg: TFMessage):

        # Look through the tf message for each of the transform datasets and store the in our 
        # class data variable
        for transform_data in msg.transforms:
            if transform_data.header.frame_id == 'odom' and transform_data.child_frame_id == 'base_footprint':
                self.path_data['tf']['base_footprint_to_odom'].append(
                    self.extract_pose_data(transform_data.transform)
            )
            if transform_data.header.frame_id == 'base_link' and transform_data.child_frame_id == 'wheel_left_link':
                self.path_data['tf']['left_wheel_to_base'].append(
                self.extract_pose_data(transform_data.transform)
            )
            if transform_data.header.frame_id == 'base_link' and transform_data.child_frame_id == 'wheel_right_link':
                self.path_data['tf']['right_wheel_to_base'].append(
                self.extract_pose_data(transform_data.transform)
            )

            self.export_to_json()

            # write data to csv file
            # df = pd.DataFrame(self.path_data)
            # df.to_csv(index=False)

        self.tf_data = msg
        # print("TF CallBack")
        # print(self.tf_data)
        # print("TF test")
        # print(self.path_data)
        # get distances with .translation and quarternions with . .rotation
    
    # need to setup the tf callback message

    def tf_static_callback(self, msg: TFMessage):
        self.tf_static_data = msg

        # print(msg[0])
        # tf_static_only contains 1 message for each of the frames
        #  hence just add the ones we want imu_link and base_link
        for transform in msg.transforms:
            # print("Transform", transform)
            if transform.header.frame_id == 'base_footprint' and transform.child_frame_id == 'base_link':
                self.path_data['tf_static']['base_link_to_base_footprint'] = self.extract_pose_data(transform.transform)
            if transform.header.frame_id == 'base_link' and transform.child_frame_id == 'imu_link':
                self.path_data['tf_static']['imu_link_to_base_link'] = self.extract_pose_data(transform.transform)

        # print("tf static:", msg)

        self.export_to_json()

        # print("TF Static CallBack")
        # print(self.tf_static_data)

    
    # need to setup the imu callback message

    def imu_callback(self, msg):
        self.imu_data = msg
        # print("IMU Data:")
        # print(self.imu_data)

        # print("IMU Message:", msg)

        # set frame ID
        self.path_data['imu']['frame'] = msg.header.frame_id

        # grab IMU data
        self.path_data['imu']['imu_data'].append({
            'rotation': {'x': msg.orientation.x,  'y': msg.orientation.y, 'z': msg.orientation.z, 'w': msg.orientation.w},
            'rotation_covariance': msg.orientation_covariance.tolist(),
            'angular_velocity': {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y, 'z': msg.angular_velocity.z},
            'angular_velocity_covariance': msg.angular_velocity_covariance.tolist(),
            'linear_acceleration': {'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y, 'z': msg.linear_acceleration.z},
            'linear_acceleration_covariance': msg.linear_acceleration_covariance.tolist()

        })

        self.export_to_json()

    # need to setup the encoder callback message

    def encoder_callback(self, msg):
        self.encoder_data = msg

        for index, encoder_name in enumerate(msg.name):

            self.path_data['encoder'][encoder_name]['position'].append(msg.position[index])
            self.path_data['encoder'][encoder_name]['velocity'].append(msg.position[index])

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    bagreader = Bagreader()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(bagreader)
    # Explicity destroys the node
    bagreader.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
