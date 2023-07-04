#!/usr/bin/env python3
import rospy

if  __name__ == '__main__':
	rospy.init_node("node_31337")

	rospy.loginfo("Node numbre 31337 is up")

	rate = rospy.Rate(10) # rate 10 Hz

	while not rospy.is_shutdown():
		rospy.loginfo("Hello")
		rate.sleep()

"""
Type of Logs
	rospy.loginfo("This is an info message")
	rospy.logwarn("This is a warm message")
	rospy.logerr("This is an error message")
"""
