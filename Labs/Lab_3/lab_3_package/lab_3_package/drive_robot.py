# import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import numpy as np

from math import pow, atan2, sqrt, asin
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
# , NavigationResult # Helper module
# from robot_navigator import BasicNavigator, NavigationResult # Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from tf2_msgs.msg import TFMessage
# quality of service for the map
from rclpy.qos import ReliabilityPolicy, QoSProfile
# imports for frame transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tabnanny import check

# Import the A* Path Planner
from . import A_Star_robot
# from A_Star_robot import Pathfinder

class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        #self.vel_publisher = self.create_publisher(Twist, '/global_costmap/costmap', 10)
        # testing with 
        # self.vel_publisher = self.create_publisher(Twist, 'turtlesim/turtle1/cmd_vel', 10)
        # self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.pose_sub = self.create_subscription(TFMessage, 'tf', self.pose_call_back, 10) # works for tf
        # testing with turtlesim
        # self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.pose = Pose()
        self.curr_pose = Pose()
        self.timer_pose = self.create_timer(0.5, self.update_current_pose)
        # self.timer = self.create_timer(0.5, self.debug_logger)
        # self.timer = self.create_timer(0.5, self.drive)
        # self.Driver 
        self.flag = False
        self.LoadedFlag = False
        # subscribe to the cost map
        # from this post best effort is the safest option to go with
        # https://piazza.com/class/l66otfiv2xt5h2/post/176
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.height = 195
        self.width = 310
        # map variables fromo map yaml file
        self.mode='trinary'
        self.resolution =0.05
        self.origin =[-5.75, -3.72, 0]
        self.negate=0
        self.occupied_thresh=0.65
        self.free_thresh=0.25

        self.goalPose = Pose()
        #  read from user
        # self.initialPose


        # vector of probabilities obtained by subscribing to the /global_costmap/costmap' topic
        self.costmap_vector = []
        
        
        self.vel_publisher  = self.create_publisher(Twist, '/cmd_vel', 10)
        # subscribing to odom is not going to work we need to do some transformations
        # self.pose_subscriber = self.create_subscription(Pose, '/odom', self.update_pose, 10)
        # self.pose_subscriber = self.create_subscription(TFMessage, 'tf', self.update_pose, 10) # works for tf
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.drive)
        # self.timer = self.create_timer(timer_period, self.debug_logger)
        # self.timer = self.create_timer(timer_period, self.update_current_pose)
        # print("Testing before timer")
        # self.timer = self.create_timer(timer_period, self.drive)
        self.pose = Pose() # pose of robot
        self.targetPose = Pose()
        self.initialPose = Pose()
        self.target_index = 0
        self.targets = []
        # = [(0.001,-1.776), (1.257,-2.309)]
        # self.targets = [(180, 120), (180, 121), (181, 121), (181, 122), (182, 122), (182, 123), (183, 123), (183, 124), (184, 124), (184, 125), (185, 125), (185, 126), (186, 126), (186, 127), (187, 127), (187, 128), (187, 129), (187, 130), (187, 131), (187, 132), (187, 133), (187, 134), (187, 135), (187, 136), (187, 137), (187, 138), (187, 139), (187, 140), (187, 141), (187, 142), (187, 143), (187, 144), (187, 145), (187, 146), (187, 147), (187, 148), (187, 149), (187, 150), (187, 151), (187, 152), (187, 153), (187, 154), (187, 155), (187, 156), (187, 157), (187, 158), (187, 159), (187, 160), (187, 161), (187, 162), (187, 163), (187, 164), (187, 165), (187, 166), (187, 167), (187, 168), (188, 168), (188, 169), (188, 170), (188, 171), (188, 172), (189, 172), (189, 173), (189, 174), (190, 174), (190, 175), (191, 175), (192, 175), (193, 175), (194, 175), (195, 175), (196, 175), (197, 175), (198, 175), (199, 175), (200, 175), (201, 175), (202, 175), (203, 175), (204, 175), (205, 175), (206, 175), (207, 175), (208, 175), (209, 175), (210, 175), (210, 174), (211, 174), (211, 173), (211, 172), (212, 172), (212, 171), (212, 170), (212, 169), (212, 168), (213, 168), (213, 167), (213, 166), (213, 165), (213, 164), (213, 163), (213, 162), (213, 161), (213, 160), (213, 159), (213, 158), (213, 157), (213, 156), (213, 155), (213, 154), (213, 153), (213, 152), (213, 151), (213, 150), (213, 149), (213, 148), (213, 147), (213, 146), (213, 145), (213, 144), (213, 143), (213, 142), (213, 141), (213, 140), (213, 139), (213, 138), (213, 137), (213, 136), (213, 135), (213, 134), (213, 133), (213, 132), (213, 131), (213, 130), (213, 129), (213, 128), (213, 127), (213, 126), (213, 125), (213, 124), (213, 123), (213, 122), (213, 121), (213, 120), (213, 119), (213, 118), (213, 117), (213, 116), (213, 115), (213, 114), (213, 113), (213, 112), (213, 111), (213, 110), (213, 109), (213, 108), (213, 107), (213, 106), (213, 105), (213, 104), (213, 103), (213, 102), (213, 101), (214, 101), (214, 100), (214, 99), (213, 99), (213, 98), (213, 97), (213, 96), (213, 95), (212, 95), (212, 94), (212, 93), (211, 93), (210, 93), (209, 93), (208, 93), (207, 93), (207, 92), (206, 92), (205, 92), (204, 92), (204, 91), (203, 91), (202, 91), (201, 91), (200, 91), (199, 91), (198, 91), (197, 91), (196, 91), (195, 91), (194, 91), (193, 91), (192, 91), (191, 91), (190, 91), (189, 91), (188, 91), (187, 91), (186, 91), (185, 91), (184, 91), (183, 91), (182, 91), (181, 91), (180, 91), (179, 91), (178, 91), (177, 91), (176, 91), (175, 91), (174, 91), (173, 91), (172, 91), (171, 91), (170, 91), (169, 91), (168, 91), (167, 91), (166, 91), (165, 91), (164, 91), (163, 91), (162, 91), (161, 91), (160, 91), (159, 91), (158, 91), (157, 91), (156, 91), (155, 91), (154, 91), (153, 91), (152, 91), (151, 91), (150, 91), (150, 92), (149, 92), (148, 92), (147, 92), (147, 93), (146, 93), (145, 93), (144, 93), (143, 93), (142, 93), (142, 94), (142, 95), (141, 95), (141, 96), (141, 97), (141, 98), (141, 99), (140, 99), (140, 100), (140, 101), (140, 102), (140, 103), (140, 104), (140, 105), (140, 106), (140, 107), (140, 108), (140, 109), (140, 110), (140, 111), (139, 111), (138, 111), (137, 111), (136, 111), (135, 111), (134, 111), (133, 111), (132, 111), (131, 111), (130, 111), (129, 111), (128, 111), (127, 111), (126, 111), (125, 111), (124, 111), (123, 111), (122, 111), (121, 111), (120, 111), (119, 111), (118, 111), (117, 111), (116, 111), (115, 111), (114, 111), (113, 111), (112, 111), (111, 111), (110, 111), (109, 111), (108, 111), (107, 111), (106, 111), (105, 111), (104, 111), (103, 111), (102, 111), (101, 111), (100, 111), (99, 111), (98, 111), (97, 111), (96, 111), (95, 111), (94, 111), (93, 111), (92, 111), (91, 111), (90, 111), (89, 111), (88, 111), (87, 111), (87, 112), (86, 112), (85, 112), (84, 112), (83, 112), (83, 113), (82, 113), (81, 113), (81, 114), (80, 114), (80, 115), (79, 115), (79, 116), (78, 116), (78, 117), (77, 117), (77, 118), (77, 119), (76, 119), (76, 120), (76, 121), (76, 122), (76, 123), (75, 123), (75, 124), (75, 125), (76, 125), (76, 126), (76, 127), (76, 128), (76, 129), (77, 129), (77, 130), (77, 131), (77, 132), (77, 133), (77, 134), (76, 134), (76, 135), (76, 136), (76, 137), (75, 137), (74, 137), (73, 137), (72, 137), (71, 137), (70, 137), (69, 137), (68, 137), (67, 137), (66, 137), (65, 137), (64, 137), (63, 137), (62, 137), (61, 137), (60, 137), (59, 137), (58, 137), (57, 137), (56, 137), (55, 137), (54, 137), (53, 137), (52, 137), (51, 137), (50, 137), (49, 137), (49, 136), (48, 136), (47, 136), (46, 136), (45, 136), (45, 135), (44, 135), (43, 135), (43, 134), (42, 134), (42, 133), (41, 133), (41, 132), (40, 132), (40, 131), (40, 130), (39, 130), (39, 129), (39, 128), (39, 127), (39, 126), (38, 126), (38, 125), (38, 124), (38, 123), (38, 122), (38, 121), (38, 120), (38, 119), (38, 118), (38, 117), (38, 116), (38, 115), (37, 115), (37, 114), (36, 114), (35, 114), (34, 114), (33, 114), (33, 113), (32, 113), (31, 113), (31, 112), (30, 112), (30, 111), (29, 111), (29, 110), (29, 109), (29, 108), (29, 107), (29, 106), (29, 105), (29, 104), (29, 103), (29, 102), (29, 101), (29, 100), (29, 99), (29, 98), (29, 97), (29, 96), (29, 95), (29, 94), (29, 93), (30, 93), (30, 92), (31, 92), (31, 91), (32, 91), (33, 91), (33, 90), (34, 90), (35, 90), (36, 90), (37, 90), (37, 89), (38, 89), (38, 88), (38, 87), (38, 86), (38, 85), (38, 84), (38, 83), (38, 82), (38, 81), (38, 80), (38, 79), (38, 78), (38, 77), (38, 76), (38, 75), (38, 74), (38, 73), (38, 72), (38, 71), (38, 70), (38, 69), (38, 68), (38, 67), (38, 66), (38, 65), (38, 64), (38, 63), (37, 63), (37, 62), (37, 61), (36, 61), (36, 60), (36, 59), (36, 58), (36, 57), (35, 57), (34, 57), (34, 56), (33, 56), (33, 55), (32, 55), (32, 54), (31, 54), (31, 53), (31, 52), (30, 52), (30, 51), (30, 50), (30, 49), (30, 48), (30, 47), (30, 46), (30, 45), (30, 44), (30, 43), (30, 42), (31, 42), (31, 41), (31, 40), (31, 39), (31, 38), (31, 37), (31, 36), (31, 35), (31, 34), (31, 33), (31, 32), (32, 32), (32, 31), (32, 30), (33, 30), (33, 29), (34, 29), (34, 28), (35, 28), (35, 27), (36, 27), (37, 27), (37, 26), (38, 26), (38, 25), (38, 24), (38, 23), (38, 22), (38, 21), (38, 20), (37, 20), (36, 20), (35, 20), (34, 20), (33, 20), (32, 20), (31, 20), (30, 20)]
        self.got_global = False
        self.lineUp = False
        self.timeConsts = [3, 1.95]
        self.timerVal = time.time()

        # Credit to tf  listener tutorial https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # set start position
        # self.update_current_pose()
        # self.timer = self.create_timer(5, self.update_current_pose())
        # # self.update_current_pose()
        # self.timer = self.create_timer(10, self.initialise())

        # self.timer = self.create_timer(timer_period, self.debug_logger)
        self.initialise()

    #  in the initialization call the Astar module and calculate the path needed

    def initialise(self):

        # Initialize starting position based on transform data
        self.startPos = self.pose
        # if self.startPos.x == 0.0: 
        #     print("Watiing for pose")
        #     # self.update_current_pose()
        #     self.startPos = self.pose
        #     print("Start pose x:", self.startPos.x)
        #     print("Start pose y:", self.startPos.y)
        #     print("pose x:", self.pose.x)
        #     print("pose y:", self.pose.y)
        #     self.timer_initial = self.create_timer(1, self.initialise())
        #     return
        
        self.startPos = self.pose
        print("Start pose x:", self.startPos.x)
        print("Start pose y:", self.startPos.y)

        # Get the input from the user.
        self.goalPose.x = float(input("Set your x goal: "))
        self.goalPose.y = float(input("Set your y goal: "))

        # if self.startPos.x == 0.0: 
        # setup Astar Path Planner
        self.get_path()


    def get_path(self):
        astar_path_planner = A_Star_robot.Pathfinder(self.width, self.height,
                                                     np.array(self.costmap_vector).reshape(-1, self.width).tolist())

        # find path pixel coordinates must be integres
        start_px = self.dist_2_px((self.startPos.x, self.startPos.y), [0, 0])
        goal_px = self.dist_2_px((self.goalPose.x, self.startPos.y), [0, 0])
        pixel_path = astar_path_planner.findPath(start_px, goal_px)

        # convert all pixel distances in map array to distance
        for index, pixel_coord in enumerate(pixel_path):
            self.targets.append((self.px_2_dist(pixel_coord, [0, 0])))

        self.targetPose.x = float(self.targets[0][0])
        self.targetPose.y = float(self.targets[0][1])

    def costmap_callback(self, costmap_msg):
        # load the cost mapdata from the subscription
        print("Printing map:")
        # set the costmap vector variable
        self.costmap_vector = costmap_msg.data
        # print(costmap_msg.data)
        self.origin = [costmap_msg.info.origin.position.x, costmap_msg.info.origin.position.y]
        self.height = costmap_msg.info.height
        self.width = costmap_msg.info.width
        self.resolution = costmap_msg.info.resolution
    
    # takes a distance x y coordinate and converts it to a pixel location
    def dist_2_px(self, coordinate, origin):
        px = round((coordinate[0] - origin[0])/(self.resolution))
        py = round((coordinate[1] - origin[1])/(self.resolution))

        return (px, py)

    def px_2_dist(self, coordinate, origin):
        print("Coordinate", coordinate)
        print("Origin", origin)
        distx = round((coordinate[0] - origin[0])*(self.resolution))
        disty = round((coordinate[1] - origin[1])*(self.resolution))

        return (distx, disty)

    
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
     
        return roll_x, pitch_y, yaw_z # in radians
    
    # get current pose by transforming base frame to map frame
    def update_current_pose(self):


        from_frame_rel = 'base_footprint'
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            
            self.pose.x = t.transform.translation.x
            self.pose.y = t.transform.translation.y
            map_rotation = t.transform.rotation
            _, _, self.pose.theta = self.euler_from_quaternion(map_rotation.x, map_rotation.y, map_rotation.z,map_rotation.w  )

            # debug testing
            self.get_logger().info(f"Current Pose: [{self.pose.x:.3f},{self.pose.y:.3f},{self.pose.theta:.3f}]")

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return


    def update_pose(self):
        # print("Pose testing", data)
        # self.pose = data
        # self.pose.x = round(self.pose.x, 4)
        # self.pose.y = round(self.pose.y, 4)
        self.update_current_pose()

        if not self.got_global:
            self.initialPose.x = self.pose.x
            self.initialPose.y = self.pose.y
            self.got_global = True

    def is_at_target(self):
        x_error = self.targetPose.x - self.pose.x
        y_error = self.targetPose.y - self.pose.y
        # range = 0.4
        range = 0.1
        if abs(x_error) < range and abs(y_error) < range:
            if self.target_index < len(self.targets):
                self.get_logger().info(
                f'Going to target{self.targets[self.target_index][0]} to {self.targets[self.target_index][1]}')
                # self.target_index += 1
                self.targetPose.x = self.targets[self.target_index][0]
                self.targetPose.y = self.targets[self.target_index][1]
                self.target_index += 1
            else:
                self.get_logger().info('Stopping')
                self.stop()

    # turn face point then start moving
    # first control angle then move

    def cal_turn_angle(self):
        x_error = self.targetPose.x - self.pose.x
        y_error = self.targetPose.y - self.pose.y
        return atan2(y_error, x_error)

    def getAngleError(self):
        # angle change that robot needs to turn to face the next point
        angle_error = self.cal_turn_angle() - self.pose.theta
        # x_error = self.targetPose.x - self.pose.x
        # y_error = self.targetPose.y - self.pose.y
        # dist = sqrt(x_error ** 2 + y_error ** 2)
        # norm_x = x_error / dist
        # norm_y = y_error / dist
        # return atan2(norm_y, norm_x)
        return angle_error

    def getDistanceError(self):
        x_error = self.targetPose.x - self.pose.x
        y_error = self.targetPose.y - self.pose.y
        dist = sqrt(x_error ** 2 + y_error ** 2)
        return dist

    def drive(self):
        if self.startPos.x == 0.0:
            print("Watiing for pose")
            self.update_pose()
            return
        elif not self.LoadedFlag:
            self.get_path()
            print("Start pose x:", self.startPos.x)
            print("Start pose y:", self.startPos.y)

            # Get the input from the user.
            self.goalPose.x = float(input("Set your x goal: "))
            self.goalPose.y = float(input("Set your y goal: "))
            self.LoadedFlag = True

        self.update_pose()
        self.is_at_target()
        vel_msg = Twist()

        k_forward = 0.1
        k_angle = 1

        vel_msg.linear.x = self.getDistanceError() * k_forward
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = self.getAngleError() * k_angle
        self.vel_publisher.publish(vel_msg)
        # debug testing
        # self.get_logger().info(f"Current Pose: [{self.pose.x:.3f},{self.pose.y:.3f},{self.pose.theta:.3f}]")
    
    def testing_move_line(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        # vel_msg.linear.y = 0.2
        #vel_msg.angular.z = -0.5
        self.vel_publisher .publish(vel_msg)
  
    def pose_call_back(self, msg):
        print("pose call back here")
        print(msg)
        print("pose call back here 2")
        # self.curr_pose.x = data.x
        # self.curr_pose.y = data.y
        # self.curr_pose.theta = data.theta
        # self.curr_pose = data
        # self.get_logger().info('X pose: "%f"' % self.curr_pose.x)
        # self.get_logger().info('Y pose: "%f"' % self.curr_pose.y)
        # self.get_logger().info('Theta pose: "%f"' % self.curr_pose.theta)
        # self.get_logger().info('Vel: "%f"' % self.curr_pose.x)
    
    def move_circle(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        vel_msg.linear.y = 0.2
        #vel_msg.angular.z = -0.5
        self.vel_publisher.publish(vel_msg)

    def move_line(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        # vel_msg.linear.y = 0.2
        #vel_msg.angular.z = -0.5
        self.vel_publisher.publish(vel_msg)
    
    def stop(self):
        # stop
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)

    # debug logger for testing
    def debug_logger(self):

        # self.get_logger().info(
        #         f' Map data resolution: {self.resolution},  origin: {self.origin}')

        # self.get_logger().info(
        #     f' Map size height: {self.height},  width: {self.width}')

        self.get_logger().info(
            f' Target Points: {self.targets}')
        # self.pose,
        # self.curr_pose
        # self.flag 
        # self.mode
        # self.resolution
        # self.origin
        # self.negate
        # self.occupied_thresh
        # self.free_thresh
        # self.costmap_vector  
        # self.targetPose
        # self.initialPose
        # self.target_index 
        # self.targets 
        # self.got_global
        # self.lineUp
        # self.timeConsts 
        # self.timerVal 
    
   
    # def euclidean_distance(self, goal_pose):
    #     return sqrt(pow((goal_pose.x - self.pose.x), 2) +
    #                 pow((goal_pose.y - self.pose.y), 2))

    # def linear_vel(self, goal_pose, constant=1.5):
    #     return constant * self.euclidean_distance(goal_pose)

    # def steering_angle(self, goal_pose):
    #     return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    # def angular_vel(self, goal_pose, constant=6):
    #     return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    # def move2goal(self):
    #     """Moves the turtle to the goal."""
    #     print("Move to goal")
    #     goal_pose = Pose()

    #     # Get the input from the user.
    #     goal_pose.x = float(input("Set your x goal: "))
    #     goal_pose.y = float(input("Set your y goal: "))

    #     # Please, insert a number slightly greater than 0 (e.g. 0.01).
    #     distance_tolerance = float(input("Set your tolerance: "))

    #     vel_msg = Twist()

    #     while self.euclidean_distance(goal_pose) >= distance_tolerance:

    #         # Porportional controller.
    #         # https://en.wikipedia.org/wiki/Proportional_control

    #         # Linear velocity in the x-axis.
    #         vel_msg.linear.x = self.linear_vel(goal_pose)
    #         vel_msg.linear.y = 0.0
    #         vel_msg.linear.z = 0.0

    #         # Angular velocity in the z-axis.
    #         vel_msg.angular.x = 0.0
        #     vel_msg.angular.y = 0.0
        #     vel_msg.angular.z = self.angular_vel(goal_pose)

        #     # Publishing our vel_msg
        #     self.vel_publisher.publish(vel_msg)

        #     # Publish at the desired rate.
        #     # self.rate.sleep()

        # # Stopping our robot after the movement is over.
        # vel_msg.linear.x = 0.0
        # vel_msg.angular.z = 0.0
        # self.vel_publisher.publish(vel_msg)
    
    def log(self):
        print(self.pose)

    

    
    # def eu_dist(self, goal):
    #     return sqrt(pow((goal.x - self.curr_pose.x), 2) + pow((goal.y - self.curr_pose.y), 2))

    # def l_vel(self, goal, kp=2):
    #     return kp*self.eu_dist(goal)

    # def head_ang(self, goal):
    #     return atan2(goal.y -self.curr_pose.y, goal.x - self.curr_pose.x)


    # def ang_vel(self, goal, kp=2):
    #     return kp*(self.head_ang(goal)- self.curr_pose.theta)

    # def reset_vel(self, vel_msg):

    #     vel_msg.linear.x = 0.0
    #     vel_msg.linear.y = 0.0
    #     vel_msg.linear.z = 0.0
    #     vel_msg.angular.z = 0.0
    #     self.vel_publisher.publish(vel_msg)

    # def draw_square_forever(self):
    #     while (True):
    #         self.draw_square()

    # # // draw square using timer

    # def draw_square(self):
    #     vel_msg = Twist()
    #     vel_msg.linear.x = 0.5
    #     vel_msg.linear.y = 0.0
    #     vel_msg.linear.z = 0.0

    #     move_time = 4
    #     turn_time = 3.04

    #     #  move 1
    #     timeout = 4    # [seconds]
    #     timeout_start = time.time()
    #     while time.time() < timeout_start + timeout:
    #         self.vel_publisher.publish(vel_msg)

    #     # // turn 1

    #     # self.reset_vel(vel_msg)
    #     vel_msg.linear.x = 0.0
    #     vel_msg.angular.z = -0.5

    #     timeout = 3.04 # [seconds]
    #     timeout_start = time.time()
    #     while time.time() < timeout_start + timeout:
    #         self.vel_publisher.publish(vel_msg)


    #     # stop
    #     vel_msg.linear.x = 0.0
    #     vel_msg.linear.y = 0.0
    #     vel_msg.linear.z = 0.0
    #     vel_msg.angular.z = 0.0
    #     self.vel_publisher.publish(vel_msg)


    #     # self.get_logger().info('X pose: "%f"' % self.curr_pose.x)
    #     # self.get_logger().info('Y pose: "%f"' % self.curr_pose.y)
    #     # self.get_logger().info('Theta pose: "%f"' % self.curr_pose.theta)

    #     # wait a second
    #     timeout = 0.1    # [seconds]
    #     timeout_start = time.time()
    #     while time.time() < timeout_start + timeout:
    #         print("Waiting")
        
        
    #     # # stop
    #     vel_msg.linear.x = 0.0
    #     vel_msg.linear.y = 0.0
    #     vel_msg.linear.z = 0.0
    #     vel_msg.angular.z = 0.0
    #     self.vel_publisher.publish(vel_msg)


def main(args=None):
    print("testing testing")
    rclpy.init(args=args)
    driver = Driver()
    # driver.stop()
    # while (True):
    #     # Tmover.draw_square_forever()
    #     driver.move_line()
    # driver.move2goal()
    # maper.log()
    # while (True):
    #     print("I have a been called Wahoo!")
    #     print('X pose: "%f"' % Tmover.curr_pose.x)


    # minimal_client = MinimalClientAsync()
    # response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    # minimal_client.destroy_node()
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()