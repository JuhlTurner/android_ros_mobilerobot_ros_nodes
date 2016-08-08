/**
 *
 * Author: Stig Turner
 *
 *
 */
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_vision/PID.h>

using namespace std;

double dist = 0.0, angle = 0.0;
ros::Subscriber sub_twist;
ros::Publisher pub_commandTwist;
double dt = 0.0;
double max_a = 0.0, min_a = 0.0, Kp_a = 0.0, Kd_a = 0.0, Ki_a = 0.0;
double max_d = 0.0, min_d = 0.0, Kp_d = 0.0, Kd_d = 0.0, Ki_d = 0.0;
PID pidAngle(dt,max_a,min_a,Kp_a,Kd_a,Ki_a);
PID pidDist(dt,max_d,min_d,Kp_d,Kd_d,Ki_d);

using namespace std;

void cb(const geometry_msgs::TwistConstPtr& msg)
{
    dist = msg->linear.x;
    angle = msg->angular.z;

    double mvdist = pidDist.calculate(0.3,dist);
    double mvangle = pidAngle.calculate(0.0,angle);

    geometry_msgs::Twist pmsg;
    pmsg.linear.x = mvdist;
    pmsg.angular.z = mvangle;
    pub_commandTwist.publish(pmsg);
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting vision control node");
    ros::init(argc, argv, "vision_control_node");
    ros::NodeHandle nh;

    sub_twist = nh.subscribe<geometry_msgs::Twist>("vision/position",1,cb);
    pub_commandTwist = nh.advertise<geometry_msgs::Twist>("control/cmd_vel",1);

    ros::spin();
	
    ROS_INFO("Terminating vision control node");
    return 0;
}
