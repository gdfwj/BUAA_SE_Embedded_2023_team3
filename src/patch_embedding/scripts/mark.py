#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import multiprocessing
from patch_embedding.srv import Base, BaseResponse
import os
import signal
from util import terminate_process


class Mark:
    def __init__(self):
        rospy.init_node("mark")
        rospy.loginfo("mark start!")

        rospy.Service('/control/mark/edit', Base, self.edit)
        rospy.Service('/control/mark/save', Base, self.save)

        self.pid = -1
        self.map_server_pid = -1

    # 编辑航点
    def edit(self, req):
        # 将当前地图的航点文件移到主目录
        mark_path = rospy.get_param("pkg_path") + '/marks/waypoints' + req.request + '.xml'
        if os.path.exists(mark_path):
            os.system("cp " + mark_path + " ~/waypoints.xml")

        if self.pid != -1:
            return BaseResponse("正在标注")

        map_path = rospy.get_param("pkg_path") + '/maps/map' + req.request + '.yaml'

        def map_server_process():
            os.system("rosrun map_server map_server " + map_path)

        p = multiprocessing.Process(target=map_server_process)
        p.start()
        self.map_server_pid = p.pid

        def t():
            os.system("roslaunch patch_embedding robot_mark.launch")
                
        p = multiprocessing.Process(target=t)
        p.start()
        self.pid = p.pid
        return BaseResponse("开始编辑")

    def save(self, req):
        mark_path = rospy.get_param("pkg_path") + '/marks/waypoints' + req.request + '.xml'
        os.system('rosrun waterplus_map_tools wp_saver')
        os.system('mv ~/waypoints.xml ' + mark_path)

        terminate_process(self.pid)
        terminate_process(self.map_server_pid)
        self.pid = -1
        self.map_server_pid = -1

        return BaseResponse("航点保存成功")

if __name__ == '__main__':
    Mark()
    rospy.spin()
