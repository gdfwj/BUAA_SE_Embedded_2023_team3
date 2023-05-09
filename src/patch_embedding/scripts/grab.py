#!/usr/bin/env python
# coding=utf-8
"""
    作者：李国玮
    时间: 2023/3/30
    机械臂抓取服务端，`/control/arm`为服务名，采用内置的Trigger作为服务类型
    运行前需启动3个自动抓取节点：`wpb_home_grab_server`、`wpb_home_grab_action`和`wpb_home_objects_3d`
"""
import os
import multiprocessing
import rospy
from std_msgs.msg import String
from patch_embedding.srv import Base, BaseResponse
from util import terminate_process

finished = False

class Grab:
    def __init__(self):
        rospy.init_node("grab_server")
        rospy.loginfo("start grab server")

        rospy.Service('/control/arm', Base, self.handle_grab)
    
    def cb_grab_result(self, msg):
        # 订阅者回调函数
        global finished
        rospy.logwarn("[GrabResultCB] %s" % msg.data)
        if msg.data == "done":
            finished = True

    def handle_grab(self, req):
        
        def t():
            if rospy.get_param("simulate"):
                os.system("roslaunch patch_embedding sim_grab.launch")
            else:
                os.system("roslaunch patch_embedding robot_grab.launch")

        p = multiprocessing.Process(target=t)
        p.start()

        # 服务函数
        global finished
        print("ready to grap")

        behaviors_pub = rospy.Publisher("/wpb_home/behaviors",String ,queue_size = 30)
        # 注意，rospy中，回调函数是单独开一个线程
        res_sub = rospy.Subscriber("/wpb_home/grab_result", String, self.cb_grab_result, queue_size=30)

        rospy.logwarn("[main] wpb_home_grab_client")
        rospy.sleep(1)
        behavior_msg = "grab start"
        # 发出自动抓取指令，抓取则由`wpb_home_grab_server`、`wpb_home_grab_action`和`wpb_home_objects_3d`3个节点协作完成
        behaviors_pub.publish(behavior_msg)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if finished == True:
                terminate_process(p.pid)
                return BaseResponse("抓取完毕")
            rate.sleep()


if __name__ == "__main__":
    Grab()
    rospy.spin()