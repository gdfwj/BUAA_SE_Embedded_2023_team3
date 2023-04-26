#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from waterplus_map_tools.srv import GetNumOfWaypoints, GetWaypointByIndex, GetWaypointByName
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient


if __name__=="__main__":
    rospy.init_node("listener_node")
    time=0

    while True:
        data = rospy.wait_for_message("cmd_vel", Twist,  queue_size=10)
        if data.linear.x == 0 and data.linear.y == 0 and data.linear.z == 0 and data.angular.x == 0 and data.angular.y == 0 and data.angular.z == 0: 
            time+=1
            if time>10: # 超时
                pose = rospy.wait_for_message("robot_pose", Pose)
                ac = SimpleActionClient("move_base", MoveBaseAction)
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = pose
                ac.send_goal(goal)
                exit(0)
        else:
            time=0
        rospy.sleep(1)