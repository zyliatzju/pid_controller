#! /usr/bin/env python

import rospy
import math
import numpy as np 

from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry as Odom
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from tf.transformations import euler_from_quaternion

class PathSubscriber():
	"""docstring for PathSubscriber"""
	def __init__(self):
		self.wp_pub = rospy.Publisher('/goal_position', Twist, queue_size = 10)
		#rospy.Subscriber("/path_planning/short_term_position", Path, self.path_cb)
		rospy.Subscriber("/global_planner_node/path", Path, self.path_cb)
		rospy.Subscriber("/amcl_pose", PoseStamped, self.amcl_cb)
		rospy.Subscriber("/goal_position_cb", Bool, self.wp_publisher_cb)
		self.path = Path()
		self.odom = Pose()
		self.new_path = False
		#self.wp_publisher() 

	def path_cb(self, msg):
		self.path = msg
		self.new_path = True
		self.wp_publisher()

	def amcl_cb(self, msg):
		self.odom = msg.pose 

	def wp_publisher(self):
		if len(self.path.poses) > 0:
			pose = self.path.poses.pop(0)
			if len(self.path.poses) > 0:
				pose = self.path.poses.pop(0)
			wp = Twist()
			wp.linear.x = pose.pose.position.x
			wp.linear.y = pose.pose.position.y
			if len(self.path.poses) > 0:
				wp.angular.z = self.get_euler_yaw(self.path.poses[-1].pose.orientation)
			else:
				wp.angular.z = self.get_euler_yaw(pose.pose.orientation)
			print "Goal: x: %.2f y: %.2f yaw: %.2f" %(wp.linear.x,wp.linear.y,wp.angular.z)


			self.wp_pub.publish(wp)

	def wp_publisher_cb(self, msg):
		if msg.data:
			self.wp_publisher()


	def get_euler_yaw(self, orientation):
		euler = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
		return euler[2]

if __name__ == '__main__':
	rospy.init_node('path_subscriber')
	ps = PathSubscriber()
	rospy.spin()
