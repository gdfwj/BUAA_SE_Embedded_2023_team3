#!/usr/bin/env python
# coding=utf-8

import os
import signal
import time
import multiprocessing
import rospy
import yaml
import psutil
import tkinter
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String
from patch_embedding.srv import Base, Conn
from util import terminate_process


tkinterUI = None


class ControllerClient:
    def __init__(self):
        self.client = rospy.ServiceProxy('/control/web', Conn)
        rospy.wait_for_service('/control/web')

    def create_map_start(self):
        resp = self.client("create_map_start", 0, "")

    def create_map_save(self, map_id=None):
        pass

    def edit_mark(self, map_id=None):
        pass

    def save_mark(self, map_id=None):
        pass

    def navigation_init(self, map_id=None):
        pass

    def navigation_begin(self, dst=None):
        pass

    def navigation_finish(self):
        pass

    def grab(self):
        pass

    def exit(self):
        terminate_process(self.init_pid)
        if rospy.get_param('use_tkinter'):
            tkinterUI.window.destroy()

class TkinterUI:
    def __init__(self, controller: ControllerClient):
        self.window = tkinter.Tk()
        self.window.geometry('400x600')
        frame = tkinter.Frame(self.window)
        frame.pack(fill='both', expand='yes')

        b1 = tkinter.Button(frame,text="开启建图",command=controller.create_map_start)
        b1.pack()
        b2 = tkinter.Button(frame,text="保存地图",command=controller.create_map_save)
        b2.pack()
        b6 = tkinter.Button(frame,text="编辑航点",command=controller.edit_mark)
        b6.pack()
        b7 = tkinter.Button(frame,text="保存航点",command=controller.save_mark)
        b7.pack()
        b3 = tkinter.Button(frame,text="进入服务模式",command=controller.navigation_init)
        b3.pack()
        b3 = tkinter.Button(frame,text="导航到目标点",command=controller.navigation_begin)
        b3.pack()
        b8 = tkinter.Button(frame,text="退出服务模式",command=controller.navigation_finish)
        b8.pack()
        b4 = tkinter.Button(frame,text="抓取",command=controller.grab)
        b4.pack()
        b5 = tkinter.Button(frame,text="退出",command=controller.exit)
        b5.pack()
        self.t = tkinter.Entry(frame)
        self.t.pack()

        l = tkinter.Label(frame, text='输出信息', font=('微软雅黑', 10, 'bold'), width=500, justify='left', anchor='w')
        l.pack()
        s1 = tkinter.Scrollbar(frame)      # 设置垂直滚动条
        s2 = tkinter.Scrollbar(frame, orient='horizontal')    # 水平滚动条
        s1.pack(side='right', fill='y')     # 靠右，充满Y轴
        s2.pack(side='bottom', fill='x')    # 靠下，充满x轴
        self.output = tkinter.Text(frame, font=('Consolas', 9), undo=True, autoseparators=False, 
            wrap='none', xscrollcommand=s2.set, yscrollcommand=s1.set)  # , state=DISABLED, wrap='none'表示不自动换行
        self.output.pack(fill='both', expand='yes')
        s1.config(command=self.output.yview)  # Text随着滚动条移动被控制移动
        s2.config(command=self.output.xview)

    def loop(self):
        self.window.mainloop()

    def log(self, text):
        now_time = time.strftime("%H:%M:%S")
        self.output.insert('end','[' + now_time + '] ' + text + '\n')



def loginfo(text):
    if rospy.get_param('use_tkinter'):
        tkinterUI.log(text.data)
    rospy.loginfo(text.data)


if __name__ == '__main__':
    rospy.init_node("controller_client")
    rospy.Subscriber("/control/logger", String, loginfo)
    
    controller = ControllerClient()
    if rospy.get_param('use_tkinter'):
        tkinterUI = TkinterUI(controller)
        tkinterUI.loop()
    rospy.spin()
    
