#!/usr/bin/env python
# coding=utf-8
import rospy
import sys
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

'''
    keyboad control model
'''
class KeyboardControl:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.x = 0.2
        self.y = 0.2
        self.z = 1
        self.move_cmd = Twist()

    def move_forward(self):
        self. move_cmd.linear.x += self.x
        self.pub.publish(self.move_cmd)

    def move_backward(self):
        self.move_cmd.linear.x -= self.x
        self.pub.publish(self.move_cmd)

    def move_left(self):
        self.move_cmd.linear.y += self.y
        self.pub.publish(self.move_cmd)

    def move_right(self):
        self.move_cmd.linear.y -= self.y
        self.pub.publish(self.move_cmd)

    def turn_left(self):
        self.move_cmd.angular.z -= self.z
        self.pub.publish(self.move_cmd)

    def turn_right(self):
        self.move_cmd.angular.z += self.z
        self.pub.publish(self.move_cmd)

    def stop(self):
        self.move_cmd.linear.x = 0
        self.move_cmd.linear.y = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)

def handle_keyboard_control(req):
    control = KeyboardControl()
    rospy.loginfo("Keyboard control enabled")

    print("键盘控制WPR机器人： \n")
    print("w - 向前加速 \n")
    print("s - 向后加速 \n")
    print("a - 向左加速 \n")
    print("d - 向右加速 \n")
    print("q - 左旋加速 \n")
    print("e - 右旋加速 \n")
    print("空格 - 刹车 \n")
    print("x - 退出 \n")
    print("------------- \n")

    while not rospy.is_shutdown():
        key = sys.stdin.read(1)
        if key == 'w':
            control.move_forward()
        elif key == 's':
            control.move_backward()
        elif key == 'a':
            control.move_left()
        elif key == 'd':
            control.move_right()
        elif key == 'q':
            control.turn_left()
        elif key == 'e':
            control.turn_right()
        elif key == ' ':
            control.stop()
        elif key == 'x':
            control.stop()
            print('keyboard input ends!\n')
            break
        else:
            rospy.loginfo("Invalid Key Command!")
    return TriggerResponse()

def keyboard_control_server():
    rospy.init_node('keyboard_control')
    s = rospy.Service('/control/keyboard', Trigger, handle_keyboard_control)
    rospy.loginfo("Keyboard control server ready")
    rospy.spin()

if __name__ == '__main__':
    keyboard_control_server()