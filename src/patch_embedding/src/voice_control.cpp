#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>   

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <geometry_msgs/Twist.h>  

geometry_msgs::Twist twist;
ros::Publisher cmd_vel_pub;
std_msgs::String string_msg;
ros::Publisher tts_text_pub;
int cmd_type = 0;

void iattextCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;

    std::string dataString = msg->data;
    if(dataString.find("前进") != dataString.npos)
    {
        char forwordString[40] = "开始前进";
        text = forwordString;
        std::cout<<text<<std::endl;
        cmd_type = 1;
    }
    else if(dataString.find("后退") != dataString.npos)
    {
        char backwordString[40] = "开始后退";
        text = backwordString;
        std::cout<<text<<std::endl;
        cmd_type = 2;
    }
    else if(dataString.find("左转") != dataString.npos)
    {
        char leftString[40] = "开始左转";
        text = leftString;
        std::cout<<text<<std::endl;
        cmd_type = 3;
    }
    else if(dataString.find("右转") != dataString.npos)
    {
        char rightdString[40] = "开始右转";
        text = rightdString;
        std::cout<<text<<std::endl;
        cmd_type = 4;
    }
    else if(dataString.find("停止") != dataString.npos)
    {
        char stopString[40] = "停止!";
        text = stopString;
        std::cout<<text<<std::endl;
        cmd_type = 5;
    }
    else
    {
        ROS_WARN("unrecognized command");
        text = msg->data.c_str();
    }

	string_msg.data = text;
	tts_text_pub.publish(string_msg);
}

int main(int argc, char* argv[])
{
    float control_speed = 0;
	float control_turn = 0;	
    ros::init(argc,argv,"voice_control");
    ros::NodeHandle n;
    ros::Subscriber iat_text_sub =n.subscribe("iat_text", 1000,iattextCallback); //subscribe voice to text reault
	tts_text_pub = n.advertise<std_msgs::String>("tts_text", 1000);  //publish text to voice string
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);  
    ROS_INFO("Wait Command...");
    // ros::spin();
    ros::Rate loop_rate(50);
    while(ros::ok()){
        switch (cmd_type)
        {
        case 1:
            control_speed = 0.1; 
            control_turn = 0.0;
            break;
        case 2:
            control_speed = -0.1; 
            control_turn = 0.0;
            break;
        case 3:
            control_speed = 0.1; 
            control_turn = 0.5;
            break;   
        case 4:
            control_speed = 0.1; 
            control_turn = -0.5;
            break;  
        case 5:
            control_speed = 0.0; 
            control_turn = 0.0;
            break;       
        default:
            break;
        }
        twist.linear.x = control_speed; 
        twist.linear.y = 0; 
        twist.linear.z = 0;
        twist.angular.x = 0; 
        twist.angular.y = 0; 
        twist.angular.z = control_turn;
        cmd_vel_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

exit:
	return 0;
}
