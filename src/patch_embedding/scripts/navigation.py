#!/usr/bin/env python2
# coding=utf-8
"""
    作者：王子豪
    时间: 2023/3/30
    导航服务端，'/control/navigator'为服务名，采用内置的Trigger作为服务类型
    需要手动设置机器人初始位置
    需要传递RunNavigator的target字段表示
    返回success表示是否成功到达
    标注放在 '~/waypoints.xml'
    运行前需启动节点：'rosrun waterplus_map_tools wp_manager'

    Waypoints采用单例模式
    节点启动后，可以通过类似test_navigation中先import，再Waypoints().getNames()的方法获取到所有的标注名称
"""

import rospy
import os
import actionlib
from actionlib import SimpleActionClient
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from patch_embedding.srv import Base, BaseResponse
import multiprocessing
from util import terminate_process

class Singleton(object):
    def __init__(self, cls):
        self._cls = cls
        self._instance = {}
    def __call__(self):
        if self._cls not in self._instance:
            self._instance[self._cls] = self._cls()
        return self._instance[self._cls]

@Singleton
class Waypoints:
    def __init__(self):
        rospy.wait_for_service("waterplus/get_num_waypoint")
        cliGetNum = rospy.ServiceProxy(
            "waterplus/get_num_waypoint", GetNumOfWaypoints)
        cliGetWPName = rospy.ServiceProxy(
            "waterplus/get_waypoint_name", GetWaypointByName)
        srvNum = cliGetNum.call()
        rospy.wait_for_service("waterplus/get_waypoint_index")
        cliGetWPIndex = rospy.ServiceProxy(
            "waterplus/get_waypoint_index", GetWaypointByIndex)
        srvl = 0
        self.sum = srvNum.num
        self.points = []
        self.dict = {}
        for i in range(srvNum.num):
            srvl = cliGetWPIndex(i)
            name = srvl.name
            self.points.append(srvl)
            self.dict[name] = srvl
    def getWaypointByName(self, target):
        return self.dict[target]
    def getNames(self):
        return self.dict.keys()


class Navigation:
    def __init__(self):
        rospy.init_node("navigation")
        rospy.loginfo("navigation start!")

        rospy.Service('/control/navigation/init', Base, self.init)
        # rospy.Service('/control/navigation/ready', Base, self.ready)
        rospy.Service('/control/navigation/begin', Base, self.begin)

        self.pid = -1
        self.map_server_pid = -1

    # 校准机器人位置
    def init(self, req):
        if self.pid != -1:
            return BaseResponse("正在校准")

        def map_server_process():
            os.system("rosrun map_server map_server " + req.request)

        p = multiprocessing.Process(target=map_server_process)
        p.start()
        self.map_server_pid = p.pid

        if rospy.get_param("simulate"):
            def t():
                os.system("roslaunch patch_embedding sim_navigation.launch")
                
            p = multiprocessing.Process(target=t)
            p.start()
            self.pid = p.pid
        return BaseResponse("开始校准")
        
    
    def begin(self, req):
        target = req.request
        srvl = Waypoints().getWaypointByName(target)
        ac = SimpleActionClient("move_base", MoveBaseAction)
        ac.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = srvl.pose
        ac.send_goal(goal)
        result = ac.wait_for_result()
        if result:
            rospy.loginfo("arrived at " + srvl.name)
            terminate_process(self.pid)
            terminate_process(self.map_server_pid)
            return BaseResponse("导航成功")
        else:
            rospy.logerr("failed to arrive at" + srvl.name)
            terminate_process(self.pid)
            terminate_process(self.map_server_pid)
            return BaseResponse("导航失败")
            
    # while True:
    #     state = ac.get_state()
    #     if state == actionlib.GoalStatus.SUCCEEDED:
    #         rospy.loginfo("Goal succeeded!")
    #         break
    #     elif state == actionlib.GoalStatus.ABORTED:
    #         rospy.loginfo("Goal aborted!")
    #         break
    #     elif state == actionlib.GoalStatus.PREEMPTED:
    #         rospy.loginfo("Goal preempted!")
    #         break
    #     else:
    #         rospy.loginfo("Goal is still active...")
        return


if __name__ == '__main__':
    Navigation()
    Waypoints()
    rospy.spin()
