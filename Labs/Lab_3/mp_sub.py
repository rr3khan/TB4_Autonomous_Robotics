# import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

print("TESTING")
from math import pow, atan2, sqrt
from nav_msgs.msg import test_OccupancyGrid



class Map_Sub(Node):

    def __init__(self):
        super().__init__('l3')
#         self.vel_publisher = self.create_publisher(Twist, '/global_costmap/costmap
# ', 10)
        #  self.vel_publisher = self.create_publisher(Twist, 'turtlesim/turtle1/cmd_vel', 10)

        self.grid = OccupancyGrid()
        self.map_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.pose_call_back, 10)
        self.map_sub 
        self.flag = False
        # Prevent unused variable warning
        # self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = AddTwoInts.Request()
    
    def get_grid(self):
        print('here Getting grid')
        self.grid = OccupancyGrid()
        self.map_sub = self.create_subscription(grid, '/global_costmap/costmap', self.pose_call_back, 10)
    
    def pose_call_back(self, data):
        print(Self.grid)
        # self.curr_pose.x = data.x
        # self.curr_pose.y = data.y
        # self.curr_pose.theta = data.theta
        # self.curr_pose = data
        # self.get_logger().info('X pose: "%f"' % self.curr_pose.x)
        # self.get_logger().info('Y pose: "%f"' % self.curr_pose.y)
        # self.get_logger().info('Theta pose: "%f"' % self.curr_pose.theta)
        # self.get_logger().info('Vel: "%f"' % self.curr_pose.x)
    
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


def main(args=None):
    print("testing testing")
    rclpy.init(args=args)
    maper = Map_Sub()
    maper.log()
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
    rclpy.spin(maper)
    rclpy.shutdown()


if __name__ == '__main__':
    main()