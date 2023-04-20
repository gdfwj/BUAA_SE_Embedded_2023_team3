#!/usr/bin/env python
# coding=utf-8
"""
    作者：李国玮
    时间: 2023/3/30
    机械臂抓取服务端，`/control/arm`为服务名，采用内置的Trigger作为服务类型
    运行前需启动3个自动抓取节点：`wpb_home_grab_server`、`wpb_home_grab_action`和`wpb_home_objects_3d`
"""
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
finished = False

def cb_grab_result(msg):
    # 订阅者回调函数
    global finished
    rospy.logwarn("[GrabResultCB] %s" % msg.data)
    if msg.data == "done":
        finished = True

def handle_grap(req):
    # 服务函数
    global finished
    print("ready to grap")

    behaviors_pub = rospy.Publisher("/wpb_home/behaviors",String ,queue_size = 30)
    # 注意，rospy中，回调函数是单独开一个线程
    res_sub = rospy.Subscriber("/wpb_home/grab_result", String, cb_grab_result, queue_size=30)

    rospy.logwarn("[main] wpb_home_grab_client")
    rospy.sleep(1)
    behavior_msg = "grab start"
    # 发出自动抓取指令，抓取则由`wpb_home_grab_server`、`wpb_home_grab_action`和`wpb_home_objects_3d`3个节点协作完成
    behaviors_pub.publish(behavior_msg)
    rate = rospy.Rate(30)
    response = TriggerResponse()

    while not rospy.is_shutdown():
        if finished == True:
            response.success = True
            response.message = "grab finished"
            return response
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("grap_server")
    rospy.logwarn("start grap_server")
    
    # 创建一个服务端，注意，在rospy中回调函数是单独一个线程运行的
    s = rospy.Service('/control/arm', Trigger, handle_grap)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()