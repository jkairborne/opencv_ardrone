#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 std::vector< cv::Point2f > corners;
int maxCorners, blockSize;
double qualityLevel, minDistance, k;
bool useHarrisDetector;

cv::Mat bw_image;
cv::Mat mask;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);


//parameters for the goodtrackingpoints algorithm
	maxCorners = 4;
	qualityLevel = 0.01;
	minDistance = 20;
	blockSize = 3;
	useHarrisDetector = false;
	k = 0.04;
	

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::cvtColor(cv_ptr->image, bw_image, cv::COLOR_BGR2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


  cv::goodFeaturesToTrack( bw_image, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

  for( size_t i = 0; i < corners.size(); i++ )
    {
    cv::circle(bw_image, corners[i], 10, cv::Scalar( 255. ), -1 );
}


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, bw_image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

