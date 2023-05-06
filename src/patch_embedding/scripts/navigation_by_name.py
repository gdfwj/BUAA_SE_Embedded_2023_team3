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
import actionlib
from actionlib import SimpleActionClient
import os
import multiprocessing
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
from embed_task5.srv import RunNavigator
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
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

def navigation(req):
    # def launch_detection():
    #     os.system("rosrun embed_task5 detect_stop.py")

    # p = multiprocessing.Process(target=launch_detection)
    
    
    target = req.target
    srvl = Waypoints().getWaypointByName(target)
    ac = SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = srvl.pose
    ac.send_goal(goal)
    # p.start()
    result = wait(ac, srvl.pose)
    # if p.is_alive():
    #     p.terminate()
    #     result = True
    # else:
    #     result = False
    if result:
        rospy.logwarn("arrived at " + srvl.name)
        s = {"success": True, "message": "arrived at " + srvl.name}
        return s
    else:
        rospy.logerr("failed")
        s = {"success": False, "message": "failed to get to  " + srvl.name}
        return s
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

def wait(ac, target_pose):
    data = None
    listener = tf.TransformListener()
    time = 0
    while True:
        try:
            data = rospy.wait_for_message("cmd_vel", Twist,  timeout=2)
        except:
            data=None
            pass
        rospy.logwarn(str(data))
        rospy.logwarn("time"+str(time))
        if  data == None or (zero(data.linear.x)and zero(data.linear.y) and zero(data.linear.z ) and zero(data.angular.x) and zero(data.angular.y) and zero(data.angular.z)): 
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
        rospy.sleep(1)

def zero(x):
    return abs(x)<1e-4

def zero2(x):
    return abs(x)<1

if __name__ == '__main__':
    rospy.init_node("navigation_by_name")
    rospy.logwarn("start navigation service")
    Waypoints()
    s = rospy.Service('/control/navigator', RunNavigator, navigation)
    rospy.spin()
