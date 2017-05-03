#include "hsvselector.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


HSVSelector::HSVSelector()
{
    std::cout << "You are in the HSVSelector constructor but with no matrix called. You may want to add an image" << '\n';
}


HSVSelector::HSVSelector(cv::Mat& image, cv::Scalar lwBd, cv::Scalar upBd, bool dispWndw, cv::Scalar lwBd2, cv::Scalar upBd2)
{

    img = image;
    lowerBd = lwBd;
    upperBd = upBd;
    lowerBd2 = lwBd2;
    upperBd2 = upBd2;
    erosion_size = 5;
    dispWindow = dispWndw;

    element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );

    cv::Mat hsv_image, result_red, result_yellow;
    cv::cvtColor(image, hsv_image, CV_BGR2HSV);

    if(dispWndw)
    {
        cv::namedWindow("Window1");

    }
    std::cout << lwBd << " " << upBd << " " << lwBd2 << " " << upBd2 << '\n';
    std::cout << BLUE_LOWER;
}


cv::Mat HSVSelector::newImage(cv::Mat &image)
{
    //So the line isn't drawn on the first image:



    // Threshold the image for the first time. If a second is required, the next code will take care of it.
    cv::Mat first_range, imgThresholded;

    cv::inRange(image, lowerBd, upperBd, first_range);

    // If a second threshold is desired (to add a separate part of the spectrum), this code will handle that
    if (lowerBd2!=SCALAR_ZERO && upperBd2!=SCALAR_ZERO)
    {
        cv::Mat second_range;
        cv::inRange(img,lowerBd2,upperBd2, second_range);

        cv::Mat first_range_copy = first_range;

        cv::addWeighted(first_range_copy, 1.0, second_range, 1.0, 0.0, first_range);
    }

    // Here we transform the image to grayscale, as well as threshold it.
    imgThresholded = first_range;

    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, element );
    dilate( imgThresholded, imgThresholded, element );

    //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, element );
    erode(imgThresholded, imgThresholded, element );


    if(dispWindow)
    {
        cv::imshow("Window 1",imgThresholded);
    }
/*
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
 */
    return imgThresholded;
}

void HSVSelector::modifyBounds1(cv::Scalar nwLwBd, cv::Scalar nwUpBd)
{
    upperBd = nwUpBd;
    lowerBd = nwLwBd;
}


void HSVSelector::modifyBounds2(cv::Scalar nwLwBd, cv::Scalar nwUpBd)
{
    upperBd2 = nwUpBd;
    lowerBd2 = nwLwBd;
}
