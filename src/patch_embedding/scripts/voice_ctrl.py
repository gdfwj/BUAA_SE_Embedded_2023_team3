#!/usr/bin/env python2
# coding=utf-8
import rospy
from patch_embedding.srv import Conn
from std_msgs.msg import String
import sys
import os

def get_text_type(text):
    if "去" in text or "渴" in text:
        ind = text.index('去')
        t = text[ind+3: len(text)-3]
        return  1, t
    elif "抓" in text:
        return 2, 0
    return 0, 0

if __name__ == "__main__":
    rospy.init_node("voice")
    args = sys.argv[:]
    text = args[1]
    type, msg = get_text_type(text)
    print(msg)
    if type==1:  # 导航前往地点
        # pub = rospy.Publisher("/tts_text", String, queue_size=1000)
        # pub.publish("开始导航")
        os.system('rostopic pub /tts_text std_msgs/String "开始导航"')
        client = rospy.ServiceProxy('/control/web', Conn)
        rospy.wait_for_service('/control/web')
        resp = client("navigation_begin", 1, "")
        # pub.publish("已经到达位置")
        os.system('rostopic pub /tts_text std_msgs/String "已经到达位置"')
        resp = client("navigation_finish", 0, "")
    elif type==2:
        # pub = rospy.Publisher("/tts_text", String, queue_size=1000)
        # pub.publish("开始抓取")
        os.system('rostopic pub /tts_text std_msgs/String "开始抓取"')
        client = rospy.ServiceProxy('/control/web', Conn)
        rospy.wait_for_service('/control/web')
        # resp = client("grab", 0, "")
        # pub.publish("抓取完成")
        os.system('rostopic pub /tts_text std_msgs/String "抓取完成"')
    else:
        try:
            # pub = rospy.Publisher("/tts_text", String, queue_size=1000)
            # pub.publish("我不理解你在说什么")
            os.system('rostopic pub /tts_text std_msgs/String "我不理解你在说什么"')
        except:
            pass
        
    
