#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include <cmath>

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<ardrone_autonomy::Navdata>("chatter", 1000);

  ros::Rate loop_rate(2);


  int count = 0;
  while (ros::ok())
  {
    ardrone_autonomy::Navdata msg;

    msg.rotX = 15;
    msg.rotY = 15*cos(count/10.0);
    msg.rotZ = 45;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


/*
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/ardrone/image_raw", 1);
  cv::Mat image = cv::imread("/home/jason/Downloads/image_square.png", CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
*/
