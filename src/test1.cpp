/*
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
*/
#include <iostream>
#include <ctype.h>
#include "ibvs.h"
#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

#define PI 3.14159265

std::vector<cv::Point2f> calc_desiredPts(cv::Point2f center, double offsetx, double offsety, double psi)
{
    Eigen::Matrix<double,2,4> vecMat, rVecMat1;

    Eigen::Matrix<double,2,2> rotMat;

    rotMat(0,0) = cos(psi*PI/180);
    rotMat(1,1) = cos(psi*PI/180);
    rotMat(1,0) = sin(psi*PI/180);
    rotMat(0,1) = -sin(psi*PI/180);

    vecMat << -offsetx,offsetx,-offsetx,offsetx,\
            -offsety,-offsety,offsety,offsety;

rVecMat1 = rotMat*vecMat;


//    std::cout << "RotationMatrix: \n" << rotMat << "\n";
    std::cout << "vecMat: \n" << vecMat << "\n";
    std::cout << "vecMat1: \n" << rVecMat1 << "\n";



    std::vector<cv::Point2f> ctr2ptVec(4);
    ctr2ptVec[0] = center+ cv::Point2f(rVecMat1(0,0),rVecMat1(1,0));
    ctr2ptVec[1] = center+ cv::Point2f(rVecMat1(0,1),rVecMat1(1,1));
    ctr2ptVec[2] = center+ cv::Point2f(rVecMat1(0,2),rVecMat1(1,2));
    ctr2ptVec[3] = center+ cv::Point2f(rVecMat1(0,3),rVecMat1(1,3));
   // desiredPts =
    return ctr2ptVec;
}

void addPtsToImg(cv::Mat& img, std::vector<cv::Point2f> ptsToAdd, cv::Scalar color)
{
    for(int i = 0; i<ptsToAdd.size(); i++)
    {
        cv::circle(img, ptsToAdd[i], 1, color, -1 );
        std::string display_string;
        std::stringstream out;
        out << i;
        display_string = out.str();

        //Add numbering to the four points discovered.
        cv::putText( img, display_string, ptsToAdd[i], CV_FONT_HERSHEY_COMPLEX, 1,color, 1, 1);
    }
}// end disp_uv

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    IBVS ibvs;
    bool correctDesiredPts;
    ros::Time start, now;
    // Function declarations
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

public:
    ImageConverter()
        : it_(nh_)
    {
        ibvs = IBVS();
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
           &ImageConverter::imageCb, this);
        start = ros::Time::now();
        correctDesiredPts = true;
    }
}; // End class


void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    cv::Mat image;
    image = cv_ptr->image;

    now = ros::Time::now();

    ros::Duration elaptime = now - start;

    std::vector<cv::Point2f> fourCorners(4);
    fourCorners =calc_desiredPts(cv::Point2f(300,150),50,20,8*elaptime.toSec());


    addPtsToImg(image,fourCorners,cv::Scalar(200,200,0));


    cv::namedWindow("Timechanger");
    cv::imshow("Timechanger", image);
    cv::waitKey(3);
}



int main(int argc, char** argv)
{
    std::cout << "now at the beginning of all";
    ros::init(argc, argv, "image_converter");
std::cout << "Ros inited";
    ImageConverter ic;

    ros::spin();
    return 0;
}

/*
#include "Eigen/Dense"
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<float, 8, 6> LeMat;


std::vector<cv::Point2f> eigenToPoint2fVector(Eigen::MatrixXd eigenMat)
{
    if (eigenMat.cols()!=1 || eigenMat.rows()%2 ) {std::cout << "eigenToPoint2fVector is receiving wrong dimension matrix - either not single column, or not even number of rows";}
    std::vector<cv::Point2f> output;

    int len = eigenMat.rows()/2;
    for(int i = 0; i< len; i++)
    {
        cv::Point2f newentry = cv::Point2f(eigenMat(2*i,0), eigenMat(2*i+1,0));

        output.push_back(newentry);
    }
    return output;
}

Eigen::MatrixXf point2fVectorToEigenVec(std::vector<cv::Point2f> pnt2fVec)
{

//customize for uv etc        if (pnt2fVec.size() == 4)
        Eigen::MatrixXf output;
        for (int i = 0; i<pnt2fVec.size(); i++)
        {
            output(2*i) = pnt2fVec[i].x;
            output(2*i+1) = pnt2fVec[i].y;
        }

        return output;
}





int main()
{
std::cout << "Just before shit goes down";

    cv::Point2f cv1 = cv::Point2f(10,20);
    cv::Point2f cv2 = cv::Point2f(20,40);
    cv::Point2f cv3 = cv::Point2f(-10,-20);
    cv::Point2f cv4 = cv::Point2f(-20,-30);
    std::vector<cv::Point2f> vecPts;
    vecPts.push_back(cv1);
    vecPts.push_back(cv2);
    vecPts.push_back(cv3);
    vecPts.push_back(cv4);
std::cout << "Just before shit goes down";
    Eigen::Matrix<float,8,1> firsteig;
    firsteig = point2fVectorToEigenVec(vecPts);

    std::cout << firsteig;


//    Le = Map<MatrixXf>(array) << endl;

    return 0;
}

*/



/*
#include <iostream>
#include "navdata_cb_ardrone.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    navdata_cb_ardrone nav = navdata_cb_ardrone();

    ros::spin();
    return 0;
}
*/


/*
 * This test was just to check a bit of vector stuff
#include "ibvs.h"
#include "posetools.h"
#include <iostream>
#include <ctype.h>

#include "ibvs.h"

#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

static const int CBOARD_COL = 5;
static const int CBOARD_ROW = 4;
int main()
{

    std::vector<cv::Point2f> abc, def;

    abc.push_back(cv::Point2f(0.2,0.3));
    abc.push_back(cv::Point2f(2.3,4.5));
    def = virtcam(abc,2, -1);

    for(int j = 0; j<def.size(); j++)
    {
        std::cout << '\n' << abc[j];
        std::cout <<" modified is: " << def[j] << " ";
    }
    std::cout <<'\n';
}
*/



/*
    IBVS def;
    def.update_Le(1.0);
    def.display_Le();
    def.MP_psinv_Le();

    std::cout << '\n' << '\n' << "Now change the pinv tolerance to 0.01: \n";
    def.update_tolerance(0.01);
    def.MP_psinv_Le();


int main()
{
    string filename = "/home/jason/Desktop/vidibvs.avi";
    VideoCapture capture(filename);
    Mat image;

      // Create IBVS object
      IBVS ibvs = IBVS();

    if( !capture.isOpened() ) {throw "Error when reading steam_avi";}

   namedWindow("w", 1);
   for( ; ; )
   {
       capture >> image;
       //if(!frame)
       //    break;
       imshow("w", image);
       waitKey(20); // waits to display frame

       int frame_width=   capture.get(CV_CAP_PROP_FRAME_WIDTH);
       int frame_height=   capture.get(CV_CAP_PROP_FRAME_HEIGHT);
       VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);    

       vector<Point2f> corners;
       Size chessSize(CBOARD_COL,CBOARD_ROW);

       std::vector<Point2f> fourCorners;

       bool patternfound = findChessboardCorners(image, chessSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
       if(patternfound)
       {
           Mat gray;
           cvtColor(image,gray,CV_BGR2GRAY);
           cornerSubPix(gray,corners,Size(11,11),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+ CV_TERMCRIT_ITER,30,0.1));

    
               // 0 4 15 19...
               fourCorners.push_back(corners[0]);
               fourCorners.push_back(corners[CBOARD_ROW]);
               fourCorners.push_back(corners[CBOARD_ROW*(CBOARD_COL-1)-1]);
               fourCorners.push_back(corners[CBOARD_COL*CBOARD_ROW-1]);
    
               ibvs.update_uv(fourCorners);
               ibvs.update_Le(1);
               Eigen::Matrix<float, 6, 1> vc;
               vc = ibvs.calculate_vc();

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
               std::string IBVS_disp_str;
               stringstream ib_str;
               ib_str << vc(0,0) << " " << vc(1,0);
               IBVS_disp_str = ib_str.str();
               cv::putText( image, IBVS_disp_str, cv::Point2f(100,100), CV_FONT_HERSHEY_COMPLEX, 1,cv::Scalar(255.), 1, 1);

         }
      video.write(image);
      imshow( "Frame", image );
      char c = (char)waitKey(33);
      if( c == 27 ) break;
   
        }
    return 0;
}

*/

/*
 *
 *
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include <algorithm>
#include <iostream>

#include "hsvselector.h"

static const std::string OPENCV_WINDOW = "Image1";


using std::vector;
using namespace cv;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    HSVSelector yellow;
    Mat mask, element, imgLines;
    bool firstTime;


    // Function declarations
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

public:
    ImageConverter()
        : it_(nh_)
    {
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1,&ImageConverter::imageCb, this);
    //    HSVSelector(cv::Scalar lwBd, cv::Scalar upBd, bool dispWndw = true, cv::Scalar lwBd2 = SCALAR_ZERO,
    //                cv::Scalar upBd2 = SCALAR_ZERO, bool dispLns = true, bool tempLns = true);

    yellow = HSVSelector(RED_LOWER,RED_UPPER,true,ORANGE_LOWER,ORANGE_UPPER);
    cv::namedWindow(OPENCV_WINDOW);

    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
}; // End class

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    ImageConverter ic;

    ros::spin();
    return 0;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    if(firstTime)
    {
        imgLines = Mat::zeros( cv_ptr->image.size(), CV_8UC3 );
        firstTime = false;
    }

    cv::Mat result = yellow.newImage(cv_ptr->image);
    //cv::imshow("Image1",result);
    cv::imshow("Image1",cv_ptr->image);
    cv::waitKey(3);


}// end ImageConverter::imageCb
*/

/*
HSV color space is also consists of 3 matrices, HUE, SATURATION and VALUE. In OpenCV, value range for  HUE, SATURATION  and VALUE  are respectively 0-179, 0-255 and 0-255. HUE represents the color, SATURATION  represents the amount to which that respective color is mixed with white and VALUE  represents the  amount to which that respective color is mixed with black.

In the above application, I have considered that the red object has HUE, SATURATION and VALUE in between 170-180, 160-255, 60-255 respectively. Here the HUE is unique for that specific color distribution of that object. But SATURATION and VALUE may be vary according to the lighting condition of that environment.

Hue values of basic colors


These are approximate values. You have to find the exact range of HUE values according to the color of the object. I found that the range of 170-179 is perfect for the range of hue values of my object. The SATURATION and VALUE is depend on the lighting condition of the environment as well as the surface of the object.

How to find the exact range of HUE, SATURATION and VALUE for a object is discussed later in this post.
*/


/*
#include "opencv2/opencv.hpp" 
#include <iostream>
#include "hsvselector.h"

using namespace cv;

int main()
{

    std::cout << "here is something";
    Mat ex = Mat::ones(100,200,CV_8U);
    cv::Scalar zeros = cv::Scalar(0,0,0);
    Scalar hundy = Scalar(100,100,100);

    HSVSelector asd = HSVSelector(ex, hundy, zeros,true , zeros, zeros);

    std::cout << std::endl;

    HSVSelector asd2 = HSVSelector();

    return 0;
}*/


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
