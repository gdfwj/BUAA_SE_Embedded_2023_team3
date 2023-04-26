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
import psutil
import tkinter
from std_srvs.srv import Trigger, TriggerRequest
from patch_embedding.srv import RunNavigator


class Controller:
    def __init__(self):

        def launch_simulation():
            os.system("roslaunch patch_embedding init.launch")
            pass

        p = multiprocessing.Process(target=launch_simulation)
        p.start()
        self.init_pid = p.pid

        # input()     # 等待launch完毕

        rospy.init_node("controller")
        self.grap_client = rospy.ServiceProxy('/control/arm', Trigger)
        self.navigation_client = rospy.ServiceProxy(
            '/control/navigator', RunNavigator)
        self.create_map_client = rospy.ServiceProxy(
            '/control/create_map', Trigger)

        self.root = tkinter.Tk()
        self.root.geometry('300x240')
        b1 = tkinter.Button(self.root,text="建图",command=self.create_map)
        b1.pack()
        b2 = tkinter.Button(self.root,text="导航",command=self.navigation)
        b2.pack()
        b3 = tkinter.Button(self.root,text="抓取",command=self.grab)
        b3.pack()
        b4 = tkinter.Button(self.root,text="退出",command=self.exit)
        b4.pack()
        self.t = tkinter.Entry(self.root)
        self.t.pack()
        self.root.mainloop()
        # while True:
        #     print("请选择一个功能（输入序号后回车）：")
        #     print("1. 建图")
        #     print("2. 导航")
        #     print("3. 抓取")
        #     print("0. 退出")
        #     try:
        #         op = int(input())
        #     except ValueError:
        #         print("输入的不是数字！")
        #         continue
        #     if op == 1:
        #         self.create_map()
        #     if op == 2:
        #         self.navigation()
        #     elif op == 3:
        #         self.grab()
        #     elif op == 0:

        #         break
        #     else:
        #         print("没有这个选项！")

    def create_map(self):

        def t():
            os.system("roslaunch patch_embedding create_map.launch")

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

        terminate_process(p.pid)
        rospy.loginfo("建图节点已关闭")

    def navigation(self):

        def t1():   # 进行导航的准备
            os.system("roslaunch patch_embedding navigation.launch")

        def t2():   # 开始导航
            os.system("rosrun patch_embedding navigation_by_name.py")

        rospy.loginfo("加载导航节点")
        p1 = multiprocessing.Process(target=t1)
        p1.start()

        input("请在校正机器人位置后按回车确认：")
        p2 = multiprocessing.Process(target=t2)
        p2.start()

        rospy.loginfo("开始导航")
        rospy.wait_for_service('/control/navigator')
        try:
            request = self.t.get()
            resp = self.navigation_client(request)
            print(resp)
        except rospy.ServiceException as e:
            print("service call failed: %s" % e)

        terminate_process(p1.pid)
        terminate_process(p2.pid)
        rospy.loginfo("导航节点已关闭")

    def grab(self):

        def t():
            os.system("roslaunch patch_embedding grab.launch")

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

        terminate_process(p.pid)
        rospy.loginfo("机械臂节点已关闭")

    def exit(self):
        terminate_process(self.init_pid)
        self.root.destroy()


def terminate_process(parent_pid):
    process = psutil.Process(parent_pid)
    children = process.children(recursive=True)
    for child in children:
        os.kill(child.pid, signal.SIGTERM)


if __name__ == '__main__':
    controller = Controller()
