#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

static const int CBOARD_COL = 5;
static const int CBOARD_ROW = 4;

using namespace cv;
using namespace std;



int main(int argc, char**argv)
{
    if(argc!=2)
    {
            cout << "Usage: chessboard imageToLoad" << endl;
            return -1;
    }


    Mat image;
    image = imread(argv[1],CV_LOAD_IMAGE_COLOR);

    if(!image.data)
    {
            cout << "Could not open or find the image" << endl;
            return -1;
    }


    vector<Point2f> corners;
    Size chessSize(CBOARD_COL,CBOARD_ROW);

    std::vector<Point2f> fourCorners;

    bool patternfound = findChessboardCorners(image, chessSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if(patternfound)
    {
        Mat gray;
        cvtColor(image,gray,CV_BGR2GRAY);
        cornerSubPix(gray,corners,Size(11,11),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+ CV_TERMCRIT_ITER,30,0.1));
    }
    // 0 4 15 19...
/*    fourCorners.push_back(corners[0]);
    fourCorners.push_back(corners[CBOARD_ROW]);
    fourCorners.push_back(corners[CBOARD_ROW*(CBOARD_COL-1)-1]);
    fourCorners.push_back(corners[CBOARD_COL*CBOARD_ROW-1]);
*/
    fourCorners.push_back(corners[0]);
    fourCorners.push_back(corners[CBOARD_COL-1]);
    fourCorners.push_back(corners[CBOARD_COL*(CBOARD_ROW-1)-1]);
    fourCorners.push_back(corners[CBOARD_ROW*CBOARD_COL-1]);
    for(int i = 0; i<4; i++)
    {
        circle(image, fourCorners[i], 1, cv::Scalar( 50. ), -1 );
    }
    cout << corners << endl;
    namedWindow("Image2");
    imshow("Image2",image);
    waitKey(0);

    return 0;
	
}
