#ifndef MOMENTFINDER_H
#define MOMENTFINDER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class MomentFinder
{
    cv::Mat img, imgLines;
    int numTempLines;
    bool linesOn, linesTemp;

    //Function declarations:
    void updateImg(cv::Mat& updImg);
public:
    MomentFinder(cv::Mat&, bool modifyimage = true, bool tempmodify = false);
    void setNumTempLines(int newNum);
    int getNumTempLines();

};

#endif // MOMENTFINDER_H
