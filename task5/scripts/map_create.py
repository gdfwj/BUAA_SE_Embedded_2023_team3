#!/usr/bin/env python
# coding=utf-8
"""
作者：刘子奇
时间：2023/4/1
建图服务端，"/control/create_map"作为服务名，
服务类型是RunMapCreater
需要在command字段给出以下两种命令中的一种
1.      Map_Start
2.      Map_Save
3.      Map_End
分别表示启动建图，保存建图，结束建图,

请注释掉wpb_scene_1中“Run the map server”语句再启动该节点
清保证embed_task5和wpr_simulation已经都放在大项目的src目录下
请保证在~/ 即主目录下没有map.yaml和map.pgm，以防被覆盖
"""

import rospy
import time
import multiprocessing
from nav_msgs.srv import GetMap
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import os
import signal


def mapCreate(req):
    rospy.loginfo("开始建图了")
    create_map_client = rospy.ServiceProxy('/control/keyboard', Trigger)
    rospy.wait_for_service('/control/keyboard')
    try:
        request = TriggerRequest()
        resp = create_map_client(request)
        print(resp)
    except:
        print('request failed')
    
    rospy.loginfo("保存地图")
    time.sleep(1)
    os.system('rosrun map_server map_saver -f map')
    time.sleep(1)
    thispath = os.path.abspath(__file__)
    dirpath = os.path.split(thispath)[0]
    savepath = os.path.join(dirpath, '../../wpr_simulation/maps/')
    os.system("mv ~/map.yaml  {}".format(savepath))
    os.system("mv ~/map.pgm  {}".format(savepath))


if __name__ == '__main__':
    rospy.init_node("map_create")
    rospy.logwarn("mapCreator start!")
    s = rospy.Service("/control/create_map", Trigger, mapCreate)
    rospy.spin()
