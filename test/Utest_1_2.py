#!/usr/bin/env python
# coding=utf-8
import rospy
import sys
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

'''
    keyboad control model
'''
class Utest_1_2:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.x = 0.2
        self.y = 0.2
        self.z = 1
        self.move_cmd = Twist()

    def test1(self):
        self. move_cmd.linear.x = 0.1
        self.move_cmd.linear.y = 0
        self.move_cmd.angular.z=0
        self.pub.publish(self.move_cmd)

    def test2(self):
        self. move_cmd.linear.x = 0
        self.move_cmd.linear.y = 0.1
        self.move_cmd.angular.z=0
        self.pub.publish(self.move_cmd)

    def test3(self):
        self. move_cmd.linear.x = 0
        self.move_cmd.linear.y = 0
        self.move_cmd.angular.z=1
        self.pub.publish(self.move_cmd)

    def test4(self):
        self. move_cmd.linear.x = 0.1
        self.move_cmd.linear.y = 0.1
        self.move_cmd.angular.z=0
        self.pub.publish(self.move_cmd)

    def test5(self):
        self. move_cmd.linear.x = 0.1
        self.move_cmd.linear.y = 0
        self.move_cmd.angular.z=1
        self.pub.publish(self.move_cmd)

    def test6(self):
        self. move_cmd.linear.x = 0.2
        self.move_cmd.linear.y = 0
        self.move_cmd.angular.z=0
        self.pub.publish(self.move_cmd)

    def stop(self):
        self.move_cmd.linear.x = 0
        self.move_cmd.linear.y = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)





if __name__ == '__main__':
        rospy.init_node("Utest_1_2")
        control = Utest_1_2()
        rospy.loginfo("Keyboard control enabled")
        while not rospy.is_shutdown():
                key = sys.stdin.read(1)
                if key == '1':
                        control.test1()
                        print (key)
                elif key == '2':
                        control.test2()
                elif key == '3':
                        control.test3()
                elif key == '4':
                        control.test4()
                elif key == '5':
                        control.test5()
                elif key == '6':
                        control.test6()
                elif key == ' ':
                        control.stop()
                elif key == 'x':
                        control.stop()
                        print('test stop!\n')
                        break
                else:
                        rospy.loginfo("Invalid Key Command!")