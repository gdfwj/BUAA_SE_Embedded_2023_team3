#!/usr/bin/env python
# coding=utf-8
"""
    作者：景泓斌
    时间: 2023/4/1
    主控程序
"""

import os
import signal
import time
import multiprocessing
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from embed_task5.srv import RunNavigator

class Controller:
    def __init__(self):
        
        def launch_simulation():
            os.system("roslaunch embed_task5 simulation.launch")

        p = multiprocessing.Process(target=launch_simulation)
        p.start()

        # input()     # 等待launch完毕

        rospy.init_node("controller")
        self.grap_client = rospy.ServiceProxy('/control/arm', Trigger)
        self.navigation_client = rospy.ServiceProxy('/control/navigator', RunNavigator)
        self.create_map_client = rospy.ServiceProxy('/control/create_map',Trigger)
    
    def ui(self):
        while True:
            print("请选择一个功能（输入序号后回车）：")
            print("1. 建图")
            print("2. 导航")
            print("3. 抓取")
            print("0. 退出")
            try:
                op = int(input())
            except ValueError:
                print("输入的不是数字！")
                continue
            if op == 1:
                self.create_map()
            if op == 2:
                self.navigation()
            elif op == 3:
                self.grab()
            elif op == 0:
                break
            else:
                print("没有这个选项！")

    def create_map(self):

        def t():
            os.system("roslaunch embed_task5 create_map.launch")

        rospy.loginfo("加载建图节点")
        p = multiprocessing.Process(target=t)
        p.start()

        rospy.loginfo("开始建图")
        rospy.wait_for_service('/control/create_map')
        try:
            request = TriggerRequest()
            resp = self.create_map_client(request)
            print(resp)
        except:
            print('request failed')
        
        p = os.kill(p.pid, signal.SIGKILL)
        rospy.loginfo("导航节点已关闭")

    def navigation(self):

        def t1():   # 进行导航的准备
            os.system("roslaunch embed_task5 navigation.launch")
        def t2():   # 开始导航
            os.system("rosrun embed_task5 navigation_by_name.py")

        rospy.loginfo("加载导航节点")
        p1 = multiprocessing.Process(target=t1)
        p1.start()
        
        input("请在校正机器人位置后按回车确认：")
        p2 = multiprocessing.Process(target=t2)
        p2.start()

        rospy.loginfo("开始导航")
        rospy.wait_for_service('/control/navigator')
        try:
            request = 'end'
            resp = self.navigation_client(request)
            print(resp)
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)
        
        os.kill(p1.pid, signal.SIGKILL)
        os.kill(p2.pid, signal.SIGKILL)
        rospy.loginfo("导航节点已关闭")

    def grab(self):

        def t():
            os.system("roslaunch embed_task5 grab.launch")

        rospy.loginfo("加载机械臂节点")
        p = multiprocessing.Process(target=t)
        p.start()

        rospy.loginfo("开始抓取")
        rospy.wait_for_service('/control/arm')
        try:
            request = TriggerRequest()
            resp = self.grap_client(request)
            print(resp.message)
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)

        os.kill(p.pid, signal.SIGKILL)
        rospy.loginfo("机械臂节点已关闭")


if __name__ == '__main__':
    controller = Controller()
    controller.ui()
