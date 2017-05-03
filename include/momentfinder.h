#ifndef MOMENTFINDER_H
#define MOMENTFINDER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class MomentFinder
{
    cv::Mat img, linesImg;
    int numTempLines, iLastX, iLastY, posX, posY;
    double dM01, dM10, dArea;
    bool linesOn, linesTemp, firstImg;

    //Function declarations:
    void drawLine();
    void calculateMoments();
public:
    MomentFinder(bool modifyimage = true, bool tempmodify = false, int nLines = 20);
    //Function declarations:
    cv::Mat updateImg(cv::Mat& updImg);
    void setNumTempLines(int newNum);
    int getNumTempLines();
    cv::Mat getImgLines();

};

#endif // MOMENTFINDER_H
