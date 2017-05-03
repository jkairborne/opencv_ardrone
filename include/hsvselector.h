#ifndef HSVSELECTOR_H
#define HSVSELECTOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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


class HSVSelector //: public MomentFinder
{
    cv::Mat hsvImg, img, resultImg, element;
    cv::Scalar lowerBd, upperBd, lowerBd2, upperBd2;
    int erosion_size;
    bool dispWindow, firstUse;

    //Function declarations

    cv::Mat hsvSelect();
public:
    HSVSelector(cv::Scalar lwBd, cv::Scalar upBd, bool dispWndw = true, cv::Scalar lwBd2 = SCALAR_ZERO, cv::Scalar upBd2 = SCALAR_ZERO);
    HSVSelector();
    cv::Mat newImage(cv::Mat &image);
    void modifyBounds1(cv::Scalar nwLwBd, cv::Scalar nwUpBd);
    void modifyBounds2(cv::Scalar nwLwBd, cv::Scalar nwUpBd);
};

#endif // HSVSELECTOR_H
