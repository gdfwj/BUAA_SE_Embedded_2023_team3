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
import time as ti
import actionlib
from actionlib import SimpleActionClient
import os
import multiprocessing
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from patch_embedding.srv import Base, BaseResponse
from util import terminate_process
from geometry_msgs.msg import Twist, Pose
import tf

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
        return self.dict[target]

    def getNames(self):
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
        return self.dict.keys()

class Navigation:
    def __init__(self):
        rospy.init_node("navigation")
        rospy.loginfo("navigation start!")

        rospy.Service('/control/navigation/init', Base, self.init)
        rospy.Service('/control/navigation/begin', Base, self.begin)
        rospy.Service('/control/navigation/finish', Base, self.finish)

        self.pid = -1
        self.map_server_pid = -1
        rospy.set_param("service_start", False)

    # 校准机器人位置
    def init(self, req):
        if self.pid != -1:
            return BaseResponse("正在校准")

        mark_path = rospy.get_param("pkg_path") + '/marks/waypoints' + str(req.request) + '.xml'
        map_path = rospy.get_param('pkg_path') + '/maps/map' + str(req.request) + '.yaml'
        if not os.path.exists(mark_path):
            return BaseResponse("航点不存在")
        elif not os.path.exists(map_path):
            return BaseResponse("地图不存在")
        rospy.set_param("service_start", True)

        os.system('cp ' + mark_path + ' ~/waypoints.xml')
        
        def map_server_process():
            os.system("rosrun map_server map_server " + map_path)

        p = multiprocessing.Process(target=map_server_process)
        p.start()
        self.map_server_pid = p.pid

        def t():
            if rospy.get_param("simulate"):
                os.system("roslaunch patch_embedding sim_navigation.launch")
            else:
                os.system("roslaunch patch_embedding robot_navigation.launch")
                
        p = multiprocessing.Process(target=t)
        p.start()
        self.pid = p.pid
        return BaseResponse("开始校准")
        
    def begin(self, req):
        if rospy.get_param("service_start") == False:
            return BaseResponse("未开启服务")
        target = req.request
        srvl = Waypoints().getWaypointByName(target)
        ac = SimpleActionClient("move_base", MoveBaseAction)
        ac.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = srvl.pose
        ac.send_goal(goal)
        result = self.wait(ac, srvl.pose)
        if result:
            rospy.loginfo("arrived at " + srvl.name)
            return BaseResponse("导航成功")
        else:
            rospy.logerr("failed to arrive at" + srvl.name)
            return BaseResponse("导航失败")

    def finish(self, req):
        if rospy.get_param("service_start") == False:
            return BaseResponse("未开启服务")
        terminate_process(self.pid)
        terminate_process(self.map_server_pid)
        self.pid = -1
        self.map_server_pid = -1
        rospy.set_param("service_start", False)
        return BaseResponse("结束服务成功")

    def wait(self, ac, target_pose):
        data = None
        listener = tf.TransformListener()
        last_trans, last_ori = None, None
        try:
            (last_trans, last_ori) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        except: 
            pass
        ti.sleep(1)
        time = 0
        while True:
            (now_trans, now_ori) = None, None
            try:
                (now_trans, now_ori) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            except:
                pass
            rospy.logwarn(str(now_trans))
            rospy.logwarn("time"+str(time))
            if  now_trans == None or last_trans == None or (zero(now_trans[0]-last_trans[0]) and zero(now_trans[1]-last_trans[1]) and zero(now_trans[2]-last_trans[2])): 
                time+=1
                if time>5: # 超时
                    trans = None
                    ori = None
                    try:
                        (trans, ori) = listener.lookupTransform("map", "base_link", rospy.Time(0))
                    except:
                        pass
                    rospy.logwarn(str(ori)+str(trans))
                    rospy.logwarn(str(target_pose))
                    if zero2(ori[0]-target_pose.orientation.x) and zero2(ori[1]-target_pose.orientation.y) and zero2(ori[2]-target_pose.orientation.z) and zero2(ori[3]-target_pose.orientation.w) and zero2(trans[0]-target_pose.position.x) and zero2(trans[1]-target_pose.position.y) and zero2(trans[2]-target_pose.position.z):
                        return True
                    else:
                        ac.cancel_all_goals()
                        return False
            else:
                time=0
            last_trans, last_ori = now_trans, now_ori
            rospy.sleep(1)

def zero(x):
    return abs(x)<1e-4

def zero2(x):
    return abs(x)<1

if __name__ == '__main__':
    Navigation()
    Waypoints()
    rospy.spin()
