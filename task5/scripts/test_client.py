#!/usr/bin/env python
# coding=utf-8
"""
    作者：李国玮
    时间: 2023/3/30
    客户端，用于向grap_server服务端发送请求，仅用于测试
"""
import rospy
from std_srvs.srv import Trigger, TriggerRequest

def send_request():
    rospy.wait_for_service('/control/arm')
    try:
        control_arm = rospy.ServiceProxy('/control/arm', Trigger)
        request = TriggerRequest()
        resp = control_arm(request)
        print(resp.message)
    except rospy.ServiceException as e:
        print("service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node("test_client")
    rospy.logwarn("start test_client")
    send_request()