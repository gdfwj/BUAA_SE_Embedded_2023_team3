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
        rospy.loginfo("开始建图")
        rospy.wait_for_service('/control/create_map')
        try:
            request = TriggerRequest()
            resp = self.create_map_client(request)
            print(resp)
        except:
            print('request failed')
        

    def navigation(self):
        request = input("请输入目标点名称：")
        rospy.loginfo("开始导航")
        rospy.wait_for_service('/control/navigator')
        try:
            resp = self.navigation_client(request)
            print(resp)
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)
        

    def grab(self):
        rospy.loginfo("开始抓取")
        rospy.wait_for_service('/control/arm')
        try:
            request = TriggerRequest()
            resp = self.grap_client(request)
            print(resp.message)
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)


if __name__ == '__main__':
    controller = Controller()
    controller.ui()
