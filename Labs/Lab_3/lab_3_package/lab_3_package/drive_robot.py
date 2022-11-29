# Credit to http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
# import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

from math import pow, atan2, sqrt
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
# , NavigationResult # Helper module
# from robot_navigator import BasicNavigator, NavigationResult # Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from tf2_msgs.msg import TFMessage

class Driver(Node):

    def __init__(self):
        super().__init__('driver')
#         self.vel_publisher = self.create_publisher(Twist, '/global_costmap/costmap
# ', 10)
        # testing with 
        # self.vel_publisher = self.create_publisher(Twist, 'turtlesim/turtle1/cmd_vel', 10)
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # self.pose_sub = self.create_subscription(TFMessage, 'tf', self.pose_call_back, 10) # works for tf
        # testing with turtlesim
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.pose = Pose()
        # self.Driver 
        self.flag = False
  
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
        vel_msg.angular.z = -0.5
        self.vel_publisher.publish(vel_msg)
    
    def stop(self):
        # stop
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)
    
    # P controller

    def update_pose(self, data):
           """Callback function which is called when a new message of type Pose is
           received by the subscriber."""
           self.pose = data
           self.pose.x = round(self.pose.x, 4)
           self.pose.y = round(self.pose.y, 4)
   
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        print("Move to goal")
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.vel_publisher.publish(vel_msg)

            # Publish at the desired rate.
            # self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)
    
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
    # driver.move_circle()
    # driver.stop()
    while (True):
        # Tmover.draw_square_forever()
        # driver.move_circle()
        driver.move2goal()
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