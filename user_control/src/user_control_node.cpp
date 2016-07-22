/**
 *
 * Author: Stig Turner
 *
 *
 */
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define updateRate 20.0


double x = 0.0, y = 0.0;
ros::Publisher pub_commandTwist;
ros::Subscriber sub_joyStickInput;

using namespace std;

void joyStickInput_cb(const sensor_msgs::Joy::ConstPtr& msg){
    x = msg->axes[0];
    y = msg->axes[1];

    double dist = sqrt(pow(x,2) + pow(y,2));
    double tmp = (y*sin(90.0*0.01745329252))/dist;
    double angle = asin(tmp);
    angle = angle - 1.57079633;
    if(x<0.0)
        angle = 0 -angle;
    ROS_INFO("Speed: %f, Angle: %f",y,angle);
    if((float)angle != (float)angle)
        angle = 0.0;
    geometry_msgs::Twist Twistmsg;
    Twistmsg.linear.x = y;
    Twistmsg.linear.y = 0.0;
    Twistmsg.linear.z = 0.0;
    Twistmsg.angular.x = 0.0;
    Twistmsg.angular.y = 0.0;
    Twistmsg.angular.z = angle;
    pub_commandTwist.publish(Twistmsg);
}

int main(int argc, char **argv)
{
    ROS_INFO("Launching user_control_node");
    ros::init(argc, argv, "user_control_node");
    ros::NodeHandle nh;
    ros::Rate rate(updateRate);
    sub_joyStickInput = nh.subscribe<sensor_msgs::Joy>("android/joyStick", 1, joyStickInput_cb);
    pub_commandTwist = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
    while(nh.ok())
    {
      ros::spinOnce();                   // Handle ROS events
      rate.sleep();
    }
	
    ROS_INFO("Terminating user_control_node");
    return 0;
}
