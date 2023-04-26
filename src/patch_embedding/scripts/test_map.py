#!/usr/bin/env python3
# coding=utf-8

"""
作者：刘子奇
时间： 2023/4/1
用于测试建图
"""
import rospy
from patch_embedding.srv import RunMapCreator

def test1():
        rospy.wait_for_service('/control/create_map')
        try:
                map = rospy.ServiceProxy('/control/create_map',RunMapCreator)
                resp= map('Map_Start')
                rospy.loginfo(resp)
        except:
               rospy.logerr('request failed')


def test2():
        rospy.wait_for_service('/control/create_map')
        try:
                map = rospy.ServiceProxy('/control/create_map',RunMapCreator)
                resp= map('Map_Save')
                rospy.loginfo(resp)
        except:
               rospy.logerr('request failed')

def test3():
        rospy.wait_for_service('/control/create_map')
        try:
                map = rospy.ServiceProxy('/control/create_map',RunMapCreator)
                resp= map('Map_End')
                rospy.loginfo(resp)
        except:
                rospy.logerr('request failed')


if __name__ == "__main__":
        rospy.init_node("test_map")
        rospy.logwarn("start test_map")
        test1()
        rate = rospy.Rate(0.01)
        rate.sleep()
        test2()
        rate = rospy.Rate(0.05)
        rate.sleep()
        #test3()