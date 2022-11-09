# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#Tutorial used to build environment: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import time
class MinimalPublisher(Node):
    def _init_(self):
        super()._init_('minimal_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
        	Pose,
        	'/odom',
        	self.update_pose,
        	10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.drive)
        #self.timer = self.create_timer(timer_period, self.drive_line)
        #self.timer = self.create_timer(timer_period, self.move_circle)
        self.pose = Pose()
        self.targetPose = Pose()
        self.initialPose = Pose()
        self.target_index = 0
        self.target_cords_x = [0.5, 0.0]
        self.target_cords_y = [0.0, 0.0]
        self.got_global = False
        self.lineUp = False
        self.timeConsts = [3, 1.95]
        self.timerVal = time.time()
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        if not self.got_global:
        	self.initialPose.x = self.pos.x
        	self.initialPose.y = self.pos.y
        	self.got_global = True
    def is_at_target(self):
        if abs(self.pose.x - self.initialPose.x - self.target_cords_x[self.target_index]) < 0.1: #and
        #abs(self.pose.y - self.initialPose.y - self.target_cords_y[self.target_index]) < 0.5):
            self.target_index += 1
        if self.target_index > 1:
            self.target_index = 0
        self.targetPose.x = round(self.target_cords_x[self.target_index], 4)
        self.targetPose.y = round(self.target_cords_y[self.target_index], 4)
    def drive_line(self):
        vel_msg = Twist()
        if time.time() > self.timerVal + self.timeConsts[int(self.lineUp)]:
            self.timerVal = time.time()
            if self.lineUp == False:
               self.lineUp = True
            elif self.lineUp == True:
               self.lineUp = False
        if self.lineUp == False:
            vel_msg.linear.y = 0.0
            vel_msg.linear.x = 0.3
        if self.lineUp == True:
            vel_msg.angular.z = 1.6
        self.pub.publish(vel_msg)
    def move_circle(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = -0.5
        self.pub.publish(vel_msg)
    def drive(self):
        #while not rospy.is_shutdown():
        self.is_at_target()
        vel_msg = Twist()
        k = 1
        x_error = self.targetPose.x - self.pose.x - self.initialPose.x
        y_error = self.targetPose.y - self.pose.y - self.initialPose.y
        vel_msg.linear.x = x_error*k
        vel_msg.linear.y = y_error*k
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.pub.publish(vel_msg)
        #msg = String()
        #msg.data = 'Hello World Josh: %d' % self.i
        #self.publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
if _name_ == '_main_':
    main()
