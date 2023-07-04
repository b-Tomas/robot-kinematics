#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ListenerNode(object):

    def __init__(self):
        rospy.init_node("listener_node")
        rospy.loginfo("[*] Listener Node Initialized")
        self.subscription_ = rospy.Subscriber('data_transfer', String, self.listener_callback, queue_size=10)

    def listener_callback(self, msg):
        rospy.loginfo("Received: " + msg.data)

    def listener(self):
        rospy.spin()

def main():
    listener_node = ListenerNode()
    listener_node.listener()

if __name__ == "__main__":
    main()
