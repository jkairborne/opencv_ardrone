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
#include "std_msgs/Int16.h"

#include <ros/ros.h>

static const int CBOARD_COL = 5;
static const int CBOARD_ROW = 4;

using namespace std;
using namespace cv;

class commandSource
{
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int currentSrc;
    // Function declarations
    void Cb(const std_msgs::Int16& msg);
public:
    commandSource()
    {
        ibvs = IBVS();
        correctDesiredPts = true;
        // Subscrive to input video feed and publish output video feed
        sub_ = nh_.subscribe("src_cmd", 5,
           &commandSource::Cb, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_ibvs", 10);
        scalingLat = 1500;
        scalingVert = 2;
        scalingPsi = 2;
    }
}; // End class

int main(int argc, char** argv)
{
    std::cout << "now at the beginning of all";
    ros::init(argc, argv, "image_converter");
std::cout << "Ros inited";
    commandSource ic;
    ros::spin();
    return 0;
}


void commandSource::processImage(Mat &image)
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


        //Want to update z_est just before calling virtual camera fct because it needs height
        ibvs.update_z_est(fourCorners);
        ibvs.update_VImPts(navdataCb.getRotM());
        ibvs.update_VCamPts();
        ibvs.addPtsToImg(image,ibvs.getVImPtsPt2F(),cv::Scalar(150,150,0));
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

void commandSource::imageCb(const std_msgs::Int16& msg)
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
