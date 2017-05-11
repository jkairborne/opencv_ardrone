#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

#include "ibvs.h"

#include "opencv2/opencv.hpp"
#include <iostream>

static const int CBOARD_COL = 5;
static const int CBOARD_ROW = 4;

using namespace std;
using namespace cv;

int main(){

    VideoCapture vcap(0);
      if(!vcap.isOpened()){
             cout << "Error opening video stream or file" << endl;
             return -1;
      }
      // Create IBVS object
      IBVS ibvs = IBVS();

   int frame_width=   vcap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height=   vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
   VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

   for(;;){
       Mat image;
       vcap >> image;

//       imshow( "Frame", image );

       vector<Point2f> corners;
       Size chessSize(CBOARD_COL,CBOARD_ROW);

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
           for(int i = 0; i<4; i++)
           {
               circle(image, fourCorners[i], 1, cv::Scalar( 50. ), -1 );
               std::string display_string;
               std::stringstream out;
               out << i;
               display_string = out.str();

               //Add numbering to the four points discovered.
               cv::putText( image, display_string, fourCorners[i], CV_FONT_HERSHEY_COMPLEX, 1,cv::Scalar(255.), 1, 1);
           }

           ibvs.update_uv(fourCorners);
           ibvs.disp_uv();
           ibvs.update_Le(1);
           ibvs.display_Le();
           ibvs.MP_psinv_Le();
    std::string IBVS_disp_str;
    stringstream ib_str;

//           cout << corners << endl;

           namedWindow("Image2");
           imshow("Image2",image);
        }

       video.write(image);
       char c = (char)waitKey(33);
       if( c == 27 ) break;
    }
  return 0;
}
