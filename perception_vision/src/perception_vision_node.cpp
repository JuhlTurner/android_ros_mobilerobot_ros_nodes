/**
 *
 * Author: Stig Turner
 *
 *
 */
#include <math.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>


//launch with "rosrun vision_control vision_control _image_transport:=compressed" to use the compressed image
//see more at : http://wiki.ros.org/image_transport/Tutorials/ExaminingImagePublisherSubscriber
using namespace cv;
using namespace std;

double x = 0.0, y = 0.0;
ros::Publisher pub_commandTwist;
image_transport::Subscriber sub_image;

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::Mat test;
    test = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("view",test);

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting vision perception node");
    ros::init(argc, argv, "vision_control_node");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    cv::startWindowThread();
    cv::Mat test;

    image_transport::ImageTransport it(nh);

    sub_image = it.subscribe("camera/image_raw", 1, imageCallback);
    pub_commandTwist = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);

    ros::spin();
	
    ROS_INFO("Terminating vision perception node");
    return 0;
}
