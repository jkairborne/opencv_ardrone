#include "opencv2/opencv.hpp" 
#include <iostream>
#include "hsvselector.h"

using namespace cv;

int main()
{
    /*
    std::cout << "here is something";
    Mat ex = Mat::ones(100,200,CV_8U);
    cv::Scalar zeros = cv::Scalar(0,0,0);
    Scalar hundy = Scalar(100,100,100);

    HSVSelector asd = HSVSelector(ex, hundy, zeros, zeros, zeros);
*/

    HSVSelector asd = HSVSelector();
    return 0;
}


/*
using namespace cv;
using namespace std;


char ky;
bool got_roi = false;
Point points_array[4];
Mat src, ROI_Img,backup,ROI_MASK;
Rect2d ROI_Select;
int width_roi = 0, height_roi = 0,min_x,min_y,max_x,max_y;
Rect ROI_RECT ;
vector< vector<Point> >  co_ordinates;

//Callback for mousclick event, the x-y coordinate of mouse button-down 
//are stored array of points [points_array].

void mouse_click(int event, int x, int y, int flags, void *param)
{

static int count=0;
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		{
			switch (count)	// number of set Point 
			{
				case 0:
					cout << "Select top-right    point" << endl;
					break;
				case 1:
					cout << "Select bottom-right point" << endl;
					break;
				case 2:
					cout << "Select bottom-left  point" << endl << endl;
					break;
				default:
					break;
			}

			if (!got_roi) // you are not select ROI yet!
			{
				points_array[count] = Point(x,y);
				circle(src, points_array[count], 2, Scalar(0, 255, 0), 2);	//show points on image
				imshow("My_Win", src);
				count++;
				if (count == 4) // if select 4 point finished
				{
					cout << "ROI x & y points :" << endl;
					cout << points_array[0] << endl;
					cout << points_array[1] << endl;
					cout << points_array[2] << endl;
					cout << points_array[3] << endl;
					cout << endl << "ROI Saved You can continue with double press any keys except 'c' " << endl <<"once press 'c' or 'C' to clear points and retry select ROI " << endl << endl;
					ky = waitKey(0) & 0xFF;
					
					if (ky == 99 || ky == 67)  // c or C to clear
					{
						backup.copyTo(src);
						points_array[0] = Point(0, 0);
						points_array[1] = Point(0, 0);
						points_array[2] = Point(0, 0);
						points_array[3] = Point(0, 0);
						imshow("My_Win", src);
						count = 0;
						cout << endl << endl << endl << "@---------------------	 Clear Points!	------------------@ " << endl << endl << endl  ;
					}
					else // user accept points & dosn't want to clear them
					{
						min_x = std::min(points_array[0].x, points_array[3].x);	//find rectangle for minimum ROI surround it!
						max_x = std::max(points_array[1].x, points_array[2].x);
						min_y = std::min(points_array[0].y, points_array[1].y);
						max_y = std::max(points_array[3].y, points_array[2].y);
						height_roi = max_y - min_y;
						width_roi = max_x - min_x;
						ROI_RECT = Rect(min_x, min_y, width_roi, height_roi);
						got_roi = true;
						co_ordinates.push_back(vector<Point>());
						co_ordinates[0].push_back(points_array[0]);
						co_ordinates[0].push_back(points_array[1]);
						co_ordinates[0].push_back(points_array[2]);
						co_ordinates[0].push_back(points_array[3]);

					}
				}
			}
			else { // if got_roi se true => select roi before
				cout << endl << "You Select ROI Before " << endl << "if you want to clear point press 'c' or double press other keys to continue" << endl << endl;
			}
			break;
		}
	}

}


int main()
{
	// replace all "My_Win" with your window name

	namedWindow("My_Win", 1);


	VideoCapture input_video("out.avi");

	// Set source imafe as [src]

	input_video >> src;
	imshow("My_Win", src);
	src.copyTo(backup);
	setMouseCallback("My_Win", mouse_click, 0);
	waitKey(0);
	Mat mask(src.rows, src.cols, CV_8UC1, cv::Scalar(0));
	drawContours(mask, co_ordinates, 0, Scalar(255), CV_FILLED, 8);

	
	while (1)
	{
		input_video >> src;
		

		//Need to copy Select ROI as MASK
		src.copyTo(ROI_MASK, mask);
		//Creat a rectangle around the Mask to reduce size of mask
		ROI_Img = ROI_MASK(ROI_RECT);


		//Show Image
		imshow("My_Win", ROI_Img);

		// Do remaining processing here on capture roi for every frame
		if(char (waitKey(1)& 0xFF) == 27) break;
			
		
	}
}
*/

/*
//One example that gets video from webcam and writes it to a video

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main(){

    VideoCapture vcap(0); 
      if(!vcap.isOpened()){
             cout << "Error opening video stream or file" << endl;
             return -1;
      }

   int frame_width=   vcap.get(CV_CAP_PROP_FRAME_WIDTH);
   int frame_height=   vcap.get(CV_CAP_PROP_FRAME_HEIGHT);
   VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

   for(;;){

       Mat frame;
       vcap >> frame;
       video.write(frame);
       imshow( "Frame", frame );
       char c = (char)waitKey(33);
       if( c == 27 ) break;
    }
  return 0;
}


// A ros example
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

//One example that gets video from webcam

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>


int main()
{

//Data Structure to store cam.
CvCapture* cap=cvCreateCameraCapture(0);
//Image variable to store frame
IplImage* frame;
//Window to show livefeed
cvNamedWindow("LiveFeed",CV_WINDOW_AUTOSIZE);

while(1)
{
    //Load the next frame
    frame=cvQueryFrame(cap);
    //If frame is not loaded break from the loop
    if(!frame)
        printf("\nno");;
    //Show the present frame
    cvShowImage("LiveFeed",frame);
    //Escape Sequence
    char c=cvWaitKey(33);
    //If the key pressed by user is Esc(ASCII is 27) then break out of the loop
    if(c==27)
       break;
}
//CleanUp
cvReleaseCapture(&cap);
cvDestroyAllWindows();
}

//Second example that gets video from webcam

#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(cvWaitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

*/
