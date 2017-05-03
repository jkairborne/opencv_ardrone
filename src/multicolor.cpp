#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <iostream>


static const std::string OPENCV_WINDOW = "Image1";
static const std::string OPENCV_WINDOW2 = "Image2"; //"Image window2";
static const std::string OPENCV_WINDOW3 = "Image3";

static const cv::Scalar SCALAR_ZERO = cv::Scalar(0,0,0);

static const cv::Scalar ORANGE_LOWER = cv::Scalar(0, 100, 100);
static const cv::Scalar ORANGE_UPPER = cv::Scalar(10, 255, 255);

static const cv::Scalar RED_LOWER = cv::Scalar(170, 100, 100);
static const cv::Scalar RED_UPPER = cv::Scalar(179, 255, 255);

static const cv::Scalar YELLOW_LOWER = cv::Scalar(22, 100, 100);
static const cv::Scalar YELLOW_UPPER = cv::Scalar(38, 255, 255);

static const cv::Scalar BLUE_LOWER = cv::Scalar(75, 100, 100);
static const cv::Scalar BLUE_UPPER = cv::Scalar(130, 255, 255);

static const cv::Scalar GREEN_LOWER = cv::Scalar(38, 100, 100);
static const cv::Scalar GREEN_UPPER = cv::Scalar(75, 255, 255);

static const cv::Scalar VIOLET_LOWER = cv::Scalar(130, 100, 100);
static const cv::Scalar VIOLET_UPPER = cv::Scalar(160, 255, 255);

using std::vector;
using namespace cv;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

    Mat mask, element, imgLines;
    int erosion_size, iLastX, iLastY;
    double dM01, dM10, dArea;
    bool firstTime;


	// Function declarations
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
    Mat hsvSelect( Mat& img, cv::Scalar lower, cv::Scalar upper, cv::Scalar lower2 = SCALAR_ZERO, cv::Scalar upper2 = SCALAR_ZERO);

public:
	ImageConverter()
		: it_(nh_)
	{

    // Set up the element for future morphological transformations.
    erosion_size = 5;
    element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );


    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1,&ImageConverter::imageCb, this);
    //image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1,&ImageConverter::imageCb, this);

	cv::namedWindow(OPENCV_WINDOW);
	cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    iLastX = -1;
    iLastY = -1;

    firstTime = true;
    }

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OPENCV_WINDOW2);
        cv::destroyWindow(OPENCV_WINDOW3);
    }
}; // End class

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");

	ImageConverter ic;

	ros::spin();
	return 0;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    if(firstTime)
    {
        imgLines = Mat::zeros( cv_ptr->image.size(), CV_8UC3 );
        firstTime = false;
    }

    cv::Mat hsv_image, result_red, result_yellow;
	cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);

    result_red = ImageConverter::hsvSelect(hsv_image,RED_LOWER,RED_UPPER,ORANGE_LOWER,ORANGE_UPPER);
    result_yellow = ImageConverter::hsvSelect(hsv_image,YELLOW_LOWER,YELLOW_UPPER);


    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW2, result_red);
    cv::waitKey(3);


}// end ImageConverter::imageCb


Mat ImageConverter::hsvSelect( Mat& img, cv::Scalar lower, cv::Scalar upper, cv::Scalar lower2, cv::Scalar upper2)
{
    //So the line isn't drawn on the first image:



    // Threshold the image. Because red wraps around the HSV scale, need two.
    cv::Mat first_range, imgBw, imgThresholded;

    cv::inRange(img, lower, upper, first_range); //0,70,75 ; 15;255;200


    if (lower2!=SCALAR_ZERO && upper2!=SCALAR_ZERO)
    {
        cv::Mat second_range;
        cv::inRange(img,lower2,upper2, second_range);

        cv::Mat first_range_copy = first_range;

        cv::addWeighted(first_range_copy, 1.0, second_range, 1.0, 0.0, first_range);
    }

    // Here we transform the image to grayscale, as well as threshold it.
//    cv::cvtColor(first_range, imgBw, CV_BGR2GRAY); //cv::COLOR_BGR2GRAY also works
//    cv::threshold(imgBw,imgThresholded,130,255,0);
    imgThresholded = first_range;

    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, element );
    dilate( imgThresholded, imgThresholded, element );

    //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, element );
    erode(imgThresholded, imgThresholded, element );


    //Calculate the moments of the thresholded image
    Moments oMoments = moments(imgThresholded);

    dM01 = oMoments.m01;
    dM10 = oMoments.m10;
    dArea = oMoments.m00;

    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
    if (dArea > 10)
    {
        ROS_INFO("In the darea callback");
     //calculate the position of the ball
     int posX = dM10 / dArea;
     int posY = dM01 / dArea;

     if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
     {
                 ROS_INFO("In the darea callback2");
      //Draw a red line from the previous point to the current point
      cv::line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
      ROS_INFO("Drawing new line: %d %d %d %d" ,posX ,posY ,iLastX ,iLastY);
     }

     iLastX = posX;
     iLastY = posY;
    }

    img = img + imgLines;
        cv::imshow(OPENCV_WINDOW3, imgLines);
    ROS_INFO("About to return imglines");
    return imgThresholded;
}


/*
HSV color space is also consists of 3 matrices, HUE, SATURATION and VALUE. In OpenCV, value range for  HUE, SATURATION  and VALUE  are respectively 0-179, 0-255 and 0-255. HUE represents the color, SATURATION  represents the amount to which that respective color is mixed with white and VALUE  represents the  amount to which that respective color is mixed with black.
　
In the above application, I have considered that the red object has HUE, SATURATION and VALUE in between 170-180, 160-255, 60-255 respectively. Here the HUE is unique for that specific color distribution of that object. But SATURATION and VALUE may be vary according to the lighting condition of that environment.

Hue values of basic colors


These are approximate values. You have to find the exact range of HUE values according to the color of the object. I found that the range of 170-179 is perfect for the range of hue values of my object. The SATURATION and VALUE is depend on the lighting condition of the environment as well as the surface of the object. 

How to find the exact range of HUE, SATURATION and VALUE for a object is discussed later in this post.
*/
