#include <string.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include<string.h>
using namespace cv;
using namespace std;

void video (void)
  {
  VideoCapture in_capture("/home/jason/Desktop/test/jan25_91/image.seq1.png");
  Mat img;

 VideoWriter out_capture("/home/jason/Desktop/test/jan25_91/video.avi", CV_FOURCC('M','J','P','G'), 30, Size(5000,5000));
cout << "pt1";
  while (true)
  {
	cout<<"pt2";
    in_capture >> img;
    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", img );
    if(img.empty())
        break;
    cout<<"pt3";
cout <<endl << img.cols << endl << img.rows;
   out_capture.write(img);
  }
}

int main()
{
cout <<"pt-1";
    video();
    cout<<"pt0";
    return 0;
}

/*
#include <string.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include<string.h>
using namespace cv;
using namespace std;


void video (void)
  {
  VideoCapture in_capture("/home/jason/Desktop/test/jan25_91/image.seq1.png");
  Mat img;

 // VideoWriter out_capture("/home/jason/Desktop/test/jan25_91/video.avi", CV_FOURCC('M','J','P','G'), 30, Size(240*3,256*3));
cout << "pt1";
  //while (true)
  //{
	cout<<"pt2";
    in_capture >> img;
    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", img );
  //  if(img.empty())
    //    break;
    cout<<"pt3";
cout <<endl << img.cols << endl << img.rows;
   // out_capture.write(img);
 // }

}

int main()
{
cout <<"pt-1";
    video();
    cout<<"pt0";
    return 0;
}
*/

