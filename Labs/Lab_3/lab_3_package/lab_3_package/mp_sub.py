# import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

print("TESTING")
from math import pow, atan2, sqrt, asin
from nav_msgs.msg import OccupancyGrid
# imports for frame transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# quality of service for the map
from rclpy.qos import ReliabilityPolicy, QoSProfile



class Map_Sub(Node):

    def __init__(self):
        super().__init__('l3_mapping')
#         self.vel_publisher = self.create_publisher(Twist, '/global_costmap/costmap
# ', 10)
        #  self.vel_publisher = self.create_publisher(Twist, 'turtlesim/turtle1/cmd_vel', 10)

        self.grid = OccupancyGrid()
        self.map_sub = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.grid_call_back, 10)
        self.map_sub 
        self.flag = False
        self.pose = Pose()
        # Prevent unused variable warning
        # self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = AddTwoInts.Request()
        # Credit to tf  listener tutorial https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer_pose = self.create_timer(0.5, self.update_current_pose)
    # def get_grid(self):
    #     print('here Getting grid')
    #     self.grid = OccupancyGrid()
    #     self.map_sub = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.grid_call_back, 10)

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

    def costmap_callback(self, costmap_msg):
        # load the cost mapdata from the subscription
        # print("Printing map:")
        # set the costmap vector variable
        self.costmap_vector = costmap_msg.data
        # print(costmap_msg.data)
        self.origin = [costmap_msg.info.origin.position.x, costmap_msg.info.origin.position.y]
        self.height = costmap_msg.info.height
        self.width = costmap_msg.info.width
        self.resolution = costmap_msg.info.resolution
    
    def grid_call_back(self, msg):
        print("Grid call back here")
        # print(msg.data)
        # print("Grid call back here 2")
        # self.curr_pose.x = data.x
        # self.curr_pose.y = data.y
        # self.curr_pose.theta = data.theta
        # self.curr_pose = data
        # self.get_logger().info('X pose: "%f"' % self.curr_pose.x)
        # self.get_logger().info('Y pose: "%f"' % self.curr_pose.y)
        # self.get_logger().info('Theta pose: "%f"' % self.curr_pose.theta)
        # self.get_logger().info('Vel: "%f"' % self.curr_pose.x)

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
    rclpy.spin(maper)
    rclpy.shutdown()


if __name__ == '__main__':
    main()