#!/usr/bin/env python
# coding=utf-8

import os
import sys
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
import asyncio
import websockets
import re

tkinterUI = None
#服务端ip地址、端口号
ip = '192.168.8.100'
# ip = 'localhost'
port = 10086

async def echo(websocket, path):
    async for message in websocket:
        print(message)
        if message=='map/create/':
            # resp = client("create_map_start", 0, "")
            controller.create_map_start()
            message = "I got your message: {}".format(message)
        elif re.match("map/save/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                controller.create_map_save(map_id=int(result.group()))
                message = "I got your message: {}".format(message)
        elif re.match("mark/create/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                controller.edit_mark(int(result.group()))
                message = "I got your message: {}".format(message)
        elif re.match("mark/save/:", message):
            map_id = message.split(":")[1]
            label_name = message.split(":")[2]
            if map_id == None or label_name == None:
                message = "Wrong, please send id"
            else :
                controller.save_mark(int(map_id), label_name)
                message = "I got your message: {}".format(message)
        elif message == 'object/fetch/':
            controller.grab()
            message = "I got your message: {}".format(message)
        elif re.match("service/init/:", message):
            result = re.search("[0-9]+", message)
            if result.group() == None:
                message = "Wrong, please send map_id"
            else :
                controller.navigation_init(int(result.group()))
                message = "I got your message: {}".format(message)
        elif re.match("navigation/begin/:", message):
            # result = re.search("[0-9]+", message)
            label_name = message.split(":")[1]
            if label_name == None:
                message = "Wrong, please send label_name"
            else :
                controller.navigation_begin(label_name)
                message = "I got your message: {}".format(message)
        elif message == 'navigation/finish/':
            controller.navigation_finish()
            message = "I got your message: {}".format(message)
        elif re.match('object/allFetch/:', message):
            label_name1 = message.split(":")[1]
            label_name2 = message.split(":")[2]
            if label_name1 == None or label_name2 == None:
                message = "Wrong, please send label_name"
            else:
                controller.fetch(label_name1, label_name2)
            message = "I got your message: {}".format(message)
        elif message == "object/pass/":
            controller.pass_obj()
            message = "I got your message: {}".format(message)
        elif message == "control/voice/":
            controller.voice()
            message = "I got your message: {}".format(message)
        else :
            message = "Invalid message!!!"
        await websocket.send(message)

class ControllerClient:
    def __init__(self):
        self.rviz_pid = -1
        self.client = rospy.ServiceProxy('/control/web', Conn)
        rospy.wait_for_service('/control/web')

    def create_map_start(self):
        p = multiprocessing.Process(target=self.rviz, args=('rviz_create_map.launch',))
        p.start()
        self.rviz_pid = p.pid
        resp = self.client("create_map_start", "0", "")

    def create_map_save(self, map_id=None):
        resp = self.client("create_map_save", str(map_id), "")
        terminate_process(self.rviz_pid)
        self.rviz_pid = -1

    def edit_mark(self, map_id=None):
        p = multiprocessing.Process(target=self.rviz, args=('rviz_mark.launch',))
        p.start()
        self.rviz_pid = p.pid
        resp = self.client("edit_mark", str(map_id), "")

    def save_mark(self, map_id=None, label=None):
        resp = self.client("save_mark", str(map_id), label)
        terminate_process(self.rviz_pid)
        self.rviz_pid = -1

    def navigation_init(self, map_id=None):
        p = multiprocessing.Process(target=self.rviz, args=('rviz_navigation.launch',))
        p.start()
        self.rviz_pid = p.pid
        resp = self.client("navigation_init", str(map_id), "")

    def navigation_begin(self, dst=None):
        resp = self.client("navigation_begin", str(dst), "")

    def navigation_finish(self):
        resp = self.client("navigation_finish", "0", "")
        terminate_process(self.rviz_pid)
        self.rviz_pid = -1

    def grab(self):
        # p = multiprocessing.Process(target=self.rviz, args=('rviz_grab.launch',))
        # p.start()
        # self.rviz_pid = p.pid
        resp = self.client("grab", "0", "")
        # terminate_process(self.rviz_pid)
        # self.rviz_pid = -1

    def pass_obj(self):
        resp = self.client("pass_obj", "0", "")

    def fetch(self, dst1, dst2):
        resp = self.client("fetch", "0", "")

    def fetch(self, label_name1,  label_name2):
        # 实现导航+取物一体化
        resp = self.client("fetch", str(label_name1), str(label_name2))

    def pass_obj(self):
        resp = self.client("pass_obj", "0", "")

    def voice(self):
        """语音服务"""
        resp = self.client("voice", "0", "")

    def exit(self):
        exit(0)
    
    def rviz(self, launch):
        os.system("roslaunch patch_embedding " + launch)

class TkinterUI:
    def __init__(self, controller: ControllerClient):
        self.window = tkinter.Tk()
        self.window.title('client')
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
        b8 = tkinter.Button(frame,text="递物",command=controller.pass_obj)
        b8.pack()
        b5 = tkinter.Button(frame,text="退出",command=controller.exit)
        b5.pack()
        self.t1 = tkinter.Entry(frame)
        self.t1.pack()
        self.t2 = tkinter.Entry(frame)
        self.t2.pack()

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
    # tkinterUI.log(text.data)
    rospy.loginfo(text.data)

def quit(signum, frame):
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node("controller_client")
    rospy.Subscriber("/control/logger", String, loginfo)
    # 实现Ctrl+C退出程序
    signal.signal(signal.SIGINT, quit)
    controller = ControllerClient()

    # 开启面板
    # def launch_tkinter():
    #     tkinterUI = TkinterUI(controller)
    #     tkinterUI.loop()
    # p = multiprocessing.Process(target=launch_tkinter)
    # p.start()

    # 注册服务端
    asyncio.get_event_loop().run_until_complete(websockets.serve(echo, ip, port))
    asyncio.get_event_loop().run_forever()
    
