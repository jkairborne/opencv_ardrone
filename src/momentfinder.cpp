#include "momentfinder.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

MomentFinder::MomentFinder(cv::Mat& inImg, bool modifyimage, bool tempmodify)
{
    img = inImg;
    linesOn = modifyimage;
    linesTemp = tempmodify;
}

void MomentFinder::updateImg(cv::Mat& updImg)
{
    img = updImg;
}

void MomentFinder::setNumTempLines(int newNum)
{
    numTempLines = newNum;
}

int MomentFinder::getNumTempLines()
{
    return numTempLines;
}
