/*
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
*/
#include <iostream>
#include <ctype.h>
#include "ibvs.h"
#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include "navdata_cb_ardrone.h"
#include "geometry_msgs/Twist.h"

static const int CBOARD_COL = 5;
static const int CBOARD_ROW = 4;

using namespace std;
using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    IBVS ibvs;
    navdata_cb_ardrone navdataCb;
    bool correctDesiredPts;
    Mat storedImage;
    geometry_msgs::Twist cmdToSend;
    int scalingLat, scalingVert, scalingPsi;
    // Function declarations
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void processImage(Mat &image);
public:
    ImageConverter(const std::string &navdataCbTopic = "/ardrone/navdata")
        : it_(nh_)
    {
        ibvs = IBVS();
        correctDesiredPts = true;
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
           &ImageConverter::imageCb, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_ibvs", 10);
        scalingLat = 500;
        scalingVert = 1;
        scalingPsi = 1;
    }
}; // End class

int main(int argc, char** argv)
{
    std::cout << "now at the beginning of all";
    ros::init(argc, argv, "image_converter");
std::cout << "Ros inited";
    ImageConverter ic;
  //  ImageConverter ic2 = ImageConverter("/chatter");

    ros::spin();
    return 0;
}


void ImageConverter::processImage(Mat &image)
{
    vector<Point2f> corners;
    Size chessSize(CBOARD_COL,CBOARD_ROW);
    namedWindow("Image2");

    bool patternfound = findChessboardCorners(image, chessSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if(patternfound)
    {
        Mat gray;
        cvtColor(image,gray,CV_BGR2GRAY);
        cornerSubPix(gray,corners,Size(11,11),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+ CV_TERMCRIT_ITER,30,0.1));

        // 0 4 15 19...
        std::vector<Point2f> fourCorners;
        fourCorners.push_back(corners[0]);
        fourCorners.push_back(corners[CBOARD_ROW]);
        fourCorners.push_back(corners[CBOARD_ROW*(CBOARD_COL-1)-1]);
        fourCorners.push_back(corners[CBOARD_COL*CBOARD_ROW-1]);
        fourCorners.resize(4);

        if(correctDesiredPts)
        {
            ibvs.rearrangeDesPts(fourCorners);
            correctDesiredPts = false;
        }

        // ibvs.calc_desiredPts(50,35);
        ibvs.addPtsToImg(image,fourCorners);
        ibvs.addPtsToImg(image,ibvs.getDesPtsPt2F(),cv::Scalar(100,100,100));
        ibvs.update_uv(fourCorners);


        //ibvs.update_z_est(0.5);
        ibvs.update_z_est(fourCorners);

        ibvs.addPtsToImg(image,ibvs.virtCam(fourCorners,navdataCb.getRotM()),cv::Scalar(150,150,0));

        ibvs.calculate_deltaS();

        ibvs.update_Le();
        ibvs.MP_psinv_Le();
        cmdToSend = ibvs.calculate_vc();

        // Manipulate and send the command:
        cmdToSend.linear.x = cmdToSend.linear.x /scalingLat;
        cmdToSend.linear.y = cmdToSend.linear.y /scalingLat;
        cmdToSend.linear.z = cmdToSend.linear.z /scalingVert;
        cmdToSend.angular.x = 0.1;
        cmdToSend.angular.y = 0;
        cmdToSend.angular.z = cmdToSend.angular.z/scalingPsi;

        pub_.publish(cmdToSend);

        imshow("Image2",image);
        cv::waitKey(3);
    }
    else
    {
    imshow("Image2",image);
    correctDesiredPts = true;
    cv::waitKey(3);
    }
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
    storedImage = cv_ptr->image;
    processImage(storedImage);
}

