#!/usr/bin/env python
# coding=utf-8
"""
    作者：王子豪
    时间: 2023/3/31
    用于测试导航客户端
"""
import rospy
from embed_task5.srv import RunNavigator
from navigation_by_name import Waypoints


def send_request():
    rospy.wait_for_service('/control/navigator')
    try:
        navigate = rospy.ServiceProxy('/control/navigator', RunNavigator)
        request = 'end'
        resp = navigate(request)
        print(resp)
    except rospy.ServiceException as e:
        print("service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("test_navigator")
    rospy.logwarn("start test_navigator_client")
    print(Waypoints().getNames())
    send_request()
