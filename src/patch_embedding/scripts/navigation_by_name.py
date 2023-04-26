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
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
from patch_embedding.srv import RunNavigator
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

def navigation(req):
    target = req.target
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


if __name__ == '__main__':
    rospy.init_node("navigation_by_name")
    rospy.logwarn("start navigation service")
    Waypoints()
    s = rospy.Service('/control/navigator', RunNavigator, navigation)
    rospy.spin()
