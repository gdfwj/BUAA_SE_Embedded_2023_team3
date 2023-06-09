import rospy
from std_msgs.msg import String
if __name__ == "__main__":
    rospy.init_node("voicectrl_test")
    pub = rospy.Publisher("tts_text_", String, queue_size=1000)
    pub_msg = String()
    pub_msg.data = text