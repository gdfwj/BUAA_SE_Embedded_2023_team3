#!/usr/bin/env python2
# coding=utf-8
import rospy
from std_msgs.msg import String
if __name__ == "__main__":
    rospy.init_node("voicectrl_test")
    pub = rospy.Publisher("voice_input_control", String, queue_size=1000)
    pub_msg = String()
    while 1:
        input()
        pub_msg.data = " "
        pub.publish(pub_msg)
    