#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";

using std::vector;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
std::vector<cv::Point2f> corners;
std::vector<cv::Point2f> old_vector;

  //std::array<cv::Point2f> corners[4];
  //std::array<cv::Point2f> old_vector[4]; 
cv::Point2f testpt;
int maxCorners, blockSize;
double qualityLevel, minDistance, k;
bool useHarrisDetector;

float prev_m_x, prev_m_y, mean_x, mean_y, counter_x, counter_y;
std::vector<cv::Point2f> vcPrevPt0, vcPrevPt1, vcPrevPt2, vcPrevPt3;
cv::Mat bw_image;
cv::Mat mask;

//int pvt_identify_pt(cv::Point2f pt_to_identify, float mean_x, float mean_y, std::vector<cv::Point2f> old_vec);

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
old_vector.resize(4);
	
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }



cv::Mat hsv_image;
cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
cv::Mat lower_red_hue_range;
cv::Mat upper_red_hue_range;
cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range); //0,70,75 ; 15;255;200
cv::inRange(hsv_image, cv::Scalar(170, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range); 

cv::Mat red_hue_mask;
cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_mask);

cv::Mat result_hsv, result_thrshld, result_dil, result_erd;

hsv_image.copyTo(result_hsv, red_hue_mask);

cv::cvtColor(result_hsv, bw_image, CV_BGR2GRAY); //cv::COLOR_BGR2GRAY
cv::threshold(bw_image,result_thrshld,130,255,0);

int erosion_size = 5; 
cv::Mat element = getStructuringElement(cv::MORPH_RECT,
              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
              cv::Point(erosion_size, erosion_size) );
	

cv::erode(result_thrshld,result_erd,element);
cv::dilate(result_erd,result_dil,element);


  cv::goodFeaturesToTrack(result_dil, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

counter_x = 0;
counter_y = 0;

  for( size_t i = 0; i < corners.size(); i++ )
    {
    cv::circle(result_dil, corners[i], 10, cv::Scalar( 255. ), -1 );
//    ROS_INFO("Corner %lu is at: %f %f", i, corners[i].y,corners[i].x);
	counter_x+=corners[i].x;
	counter_y+=corners[i].y;
    }

mean_x = counter_x/corners.size();
mean_y = counter_y/corners.size();
//testpts[0].x = mean_x;
//testpts[0].y = mean_y;
testpt.x = mean_x;
testpt.y = mean_y;
cv::circle(result_dil, testpt,15,cv::Scalar(255.));


//Now, we need to identify the four squares:
// Have created the pvt_..._... function to handle this. Essentially, it will check which of the four squares is closest to which for the next iteration.



//Now populate the old vector, for the next iteration
for(std::vector<cv::Point2f>::size_type i = 0; i < corners.size(); i++) {
//ROS_INFO("IN FOR LOOP %d, %d", i, corners.size());
  //  old_vector.push_back(cv::Point2f(corners[i].y-mean_y,corners[i].x-mean_x)); // This works too
  old_vector[i] = cv::Point2f(corners[i].y-mean_y,corners[i].x-mean_x);
}

/*
for(std::vector<cv::Point2f>::size_type i = 0; i <= corners.size(); i++)
    {
// x and y for CV Mats are not the "x,y" we think of for images, but rather more like matrices - i.e. "y" first, then "x". This may be wrong.
	old_vector[i] = cv::Point2f(corners[i].y-mean_y,corners[i].x-mean_x);
    }
*/
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW2, result_erd);
    cv::imshow(OPENCV_WINDOW, result_dil);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }// end callback
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

/*int pvt_identify_pt(cv::Point2f pt_to_identify, float mean_x, float mean_y, std::vector<cv::Point2f> old_vec)
{
std::vector<cv::Point2f> current_dist;
int j = 0;
for (j=0;j<4;j++)
{
	current_dist[j].x = (old_vec[j].x - pt_to_identify.x) + (old_vec[j].y - pt_to_identify.y);
	current_dist[j].y = j;
}//end for*/

/*
int pvt_identify_pt(cv::Point2f pt_to_identify, float mean_x, float mean_y, std::vector<cv::Point2f> old_vec)
{
std::vector<int> current_dist;
int j = 0;
int index_min = 0;
for (j=0;j<4;j++)
{
	current_dist[j] = (old_vec[j].x - pt_to_identify.x) + (old_vec[j].y - pt_to_identify.y);
	if(current_dist[j] < current_dist[index_min]) 
		index_min = j;
}//end for

	return index_min;
}
*/
