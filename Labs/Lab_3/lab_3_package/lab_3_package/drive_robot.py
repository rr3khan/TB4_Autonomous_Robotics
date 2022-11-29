# import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

print("TESTING")
from math import pow, atan2, sqrt
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
# , NavigationResult # Helper module
# from robot_navigator import BasicNavigator, NavigationResult # Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp


class Driver(Node):

    def __init__(self):
        super().__init__('driver')
#         self.vel_publisher = self.create_publisher(Twist, '/global_costmap/costmap
# ', 10)
        #  self.vel_publisher = self.create_publisher(Twist, 'turtlesim/turtle1/cmd_vel', 10)

        # self.pose = Twist()
        self.pose = self.create_subscription(Pose,
        	'/odom',
        	self.got_to_goal,
        	10)
        self.flag = False
        # Prevent unused variable warning
        # self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = AddTwoInts.Request()
    
    # def get_pose(self):
    #     print('here Getting grid')
    #     self.grid = OccupancyGrid()
    #     self.Driver = self.create_subscription(Pose,
    #     	'/odom',
    #     	self.update_pose,
    #     	10)
    
    def get_pose(self, data):
        print(data)
        print("Printing pose")
    
        # P controller
    
    def log(self):
        print(self.grid)
    
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


# https://automaticaddison.com/go-to-a-goal-location-upon-low-battery-ros-2-navigation/

    def got_to_goal(self):
        # Launch the ROS 2 Navigation Stack
        navigator = BasicNavigator()
    
        # Wait for navigation to fully activate. Use this line if autostart is set to true.
        navigator.waitUntilNav2Active()
    
        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')
    
        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
    
        # Set the robot's goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.position.z = 0.25
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
    
        # Go to the goal pose
        navigator.goToPose(goal_pose)
    
        i = 0
    
        # Keep doing stuff as long as the robot is moving towards the goal
        while not navigator.isNavComplete():
            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                feedback.distance_remaining) + ' meters.')
        
            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()
    
        # Do something depending on the return code
        #   result = navigator.getResult()
        #   if result == NavigationResult.SUCCEEDED:
        #     print('Successfully reached charging dock staging area...')
        #     low_battery = False
        #     self.connect_to_dock()
        #   elif result == NavigationResult.CANCELED:
        #     print('Goal was canceled!')
        #   elif result == NavigationResult.FAILED:
        #     print('Goal failed!')
        #   else:
        #     print('Goal has an invalid return status!') 


def main(args=None):
    print("testing testing")
    rclpy.init(args=args)
    driver = Driver()
    # maper.log()
    print("I have a been called Wahoo!")
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