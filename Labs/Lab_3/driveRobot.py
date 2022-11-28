#Credit to: http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal for class structure and pose gathering callback
#https://raw.githubusercontent.com/ros/ros_tutorials/noetic-devel/rospy_tutorials/#001_talker_listener/talker.py for publisher

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
from math import pow, atan2, sqrt

class robotDriver():
	def __init__(self):
		rospy.init_node('robot_driver', anonymous=True)
		self.pub = rospy.Publisher('/turtlesim/turtle1/cmd_vel', Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber('/turtlesim/turtle1/pose', Pose, self.update_pose)
		self.server = rospy.Service('updateTargetPose', AddTwoInts, self.update_target_pose)
		self.pose = Pose()
		self.targetPose = Pose()
		self.targetPose.x = 2
		self.targetPose.y = 2
	   
	def update_pose(self, data):
	
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)
	
	def update_target_pose(self, req):
		print("Request")
		print(req)
		self.targetPose.x = round(req.x, 4)
		self.targetPose.y = round(req.y, 4)
		return AddTwoIntsResponse(0)

	def getAngleError(self):
		e_x =

	def drive(self):
		while not rospy.is_shutdown():
			vel_msg = Twist()
			k = 1
			x_error = self.targetPose.x - self.pose.x
			y_error = self.targetPose.y - self.pose.y
			rospy.loginfo(x_error)
			vel_msg.linear.x = x_error*k
			vel_msg.linear.y = y_error*k
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0
			rate = rospy.Rate(1)    
			self.pub.publish(vel_msg)
			rate.sleep()

if __name__ == '__main__':
	try:
		driver = robotDriver()
		driver.drive()
	except rospy.ROSInterruptException:
		pass
