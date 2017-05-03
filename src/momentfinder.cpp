#include "momentfinder.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

MomentFinder::MomentFinder(bool modifyimage, bool tempmodify, int nLines)
{
    linesOn = modifyimage;
    linesTemp = tempmodify;
    numTempLines = nLines;
    iLastX = -1;
    iLastY = -1;
    posX = 0;
    posY = 0;
    firstImg = true;
}

cv::Mat MomentFinder::updateImg(cv::Mat& updImg)
{
    if(firstImg)
    {
        linesImg = cv::Mat(updImg.size(),updImg.type());
        firstImg = false;
    }
    img = updImg;
    calculateMoments();
    drawLine();
    return img;
}

void MomentFinder::calculateMoments()
{
    //Calculate the moments of the thresholded image
    cv::Moments oMoments = cv::moments(img);

    dM01 = oMoments.m01;
    dM10 = oMoments.m10;
    dArea = oMoments.m00;
}


void MomentFinder::drawLine()
{
    // if the area <= 100, I consider that the there are no object in the image and it's because of the noise, the area is not zero
    if (dArea > 100)
    {
     //calculate the position of the ball
     int posX = dM10 / dArea;
     int posY = dM01 / dArea;

     if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
     {
         if (linesTemp)
         {
             std::cout << "Here there should be a vector of lines to use..";
         }
         else
         {
              //Draw a line from the previous point to the current point
              cv::line(linesImg, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(255,255,255), 2);
              std::cout << "posX: " << posX << " posY: " << posY << " iLastX: " << iLastX << " iLastY: " << iLastY << '\n';
              cv::namedWindow("testing");
              cv::imshow("testing",linesImg);
         }
     }
     iLastX = posX;
     iLastY = posY;
    }

    if(linesOn)
    {
        img = img + linesImg;
    }
} // end drawLine

void MomentFinder::setNumTempLines(int newNum)
{
    numTempLines = newNum;
}

int MomentFinder::getNumTempLines()
{
    return numTempLines;
}
