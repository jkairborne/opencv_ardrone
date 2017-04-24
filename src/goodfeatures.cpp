#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "corrected"; //"Image window2";
static const int NUMPTS = 4;

// function declarations
float distance_calc(cv::Point2f, cv::Point2f);
int within_bounds(int index);


using std::vector;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	int maxCorners, blockSize;
	double qualityLevel, minDistance, k;
	bool useHarrisDetector;

	float prev_m_x, prev_m_y, mean_x, mean_y, counter_x, counter_y;
	std::vector<cv::Point2f> corners, old_vector, vcPrevPt0, vcPrevPt1, vcPrevPt2, vcPrevPt3;
	cv::Point2f MeanPt;
	cv::Mat bw_image;
	cv::Mat mask;
	
	int ptarray[NUMPTS];
	

	// Function declarations
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	std::vector<cv::Point2f> pvt_identify_pt(std::vector<cv::Point2f> pts_to_identify, std::vector<cv::Point2f> old_vec);

public:
	ImageConverter()
		: it_(nh_)
	{
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

	//parameters for the goodtrackingpoints algorithm
	maxCorners = NUMPTS;
	qualityLevel = 0.01;
	minDistance = 20;
	blockSize = 3;
	useHarrisDetector = false;
	k = 0.04;
	//We want old vector to have size of number of points, plus one for the mean
	old_vector.resize(NUMPTS+1);
	
	cv::namedWindow(OPENCV_WINDOW);
	cv::namedWindow(OPENCV_WINDOW2);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow(OPENCV_WINDOW2);
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

	cv::Mat hsv_image;
	cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
	cv::Mat lower_red_hue_range;
	cv::Mat upper_red_hue_range;

	// Threshold the image. Because red wraps around the HSV scale, need two.
	cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range); //0,70,75 ; 15;255;200
	cv::inRange(hsv_image, cv::Scalar(170, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range); 

	cv::Mat red_hue_mask;
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_mask);

	cv::Mat result_hsv, result_thrshld, result_dil, result_erd;

	hsv_image.copyTo(result_hsv, red_hue_mask);


	// Here we transform the image to grayscale, as well as threshold it.
	cv::cvtColor(result_hsv, bw_image, CV_BGR2GRAY); //cv::COLOR_BGR2GRAY also works
	cv::threshold(bw_image,result_thrshld,130,255,0);

	// Set up the element for future morphological transformations.
	int erosion_size = 5; 
	cv::Mat element = getStructuringElement(cv::MORPH_RECT,
	cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
	cv::Point(erosion_size, erosion_size) );
	
	// Use Morphological transformations. This should get rid of the noise at the edges of the rectangle.
	// The "element" is in our case a rectangle, leading to smooth edges.
	cv::erode(result_thrshld,result_erd,element);
	cv::dilate(result_erd,result_dil,element);

	// Use OpenCV's goodFeaturesToTrack algorithm to find just that - good corners to be tracked.
	cv::goodFeaturesToTrack(result_dil, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

	//Now, we need to identify which of the four corners is which:
	// Have created the pvt_..._... function to handle this. Essentially, it will check which of the four squares is closest to which for the next iteration.
	// Essentially, we want to re-assign values of corners for the 
std::vector<cv::Point2f> test2;
test2.resize(4);

test2 = pvt_identify_pt(corners,old_vector);

	//counters for calculating the mean of all four points.
	counter_x = 0;
	counter_y = 0;

	for( size_t i = 0; i < corners.size(); i++ )
	{
		// Add a circle around the detected corners
		cv::circle(result_dil, corners[i], 10, cv::Scalar( 50. ), -1 );
		cv::circle(result_dil, test2[i], 10, cv::Scalar( 50. ), -1 );
		cv::circle(result_dil, old_vector[i], 10, cv::Scalar( 50. ), -1 );

		// Create a string with just the number we would like to display.
		std::string display_string;
		std::stringstream out;
		out << i;
		display_string = out.str();

		//Add numbering to the four points discovered.
		cv::putText( result_dil, display_string, corners[i], CV_FONT_HERSHEY_COMPLEX, 1,cv::Scalar(255.), 1, 1);
		cv::putText( result_dil, display_string, test2[i], CV_FONT_HERSHEY_COMPLEX, 1,cv::Scalar(255.), 1, 1);


		//    ROS_INFO("Corner %lu is at: %f %f", i, corners[i].y,corners[i].x);
		counter_x+=corners[i].x;
		counter_y+=corners[i].y;
	} // end for

	// Calculate the mean location of the points.
	mean_x = counter_x/corners.size();
	mean_y = counter_y/corners.size();
	MeanPt = cv::Point2f(mean_x,mean_y);
	cv::circle(result_dil, MeanPt,15,cv::Scalar(100.));

	//Now populate the old vector, for the next iteration
	for(std::vector<cv::Point2f>::size_type i = 0; i < corners.size(); i++) 
	{
	// x and y for CV Mats are not the "x,y" we think of for images, but rather more like matrices - i.e. "y" first, then "x". This may be wrong.
		  old_vector[i] = cv::Point2f(corners[i].y-mean_y,corners[i].x-mean_x);
	} // end for
	old_vector[NUMPTS+1] = MeanPt;

	// Update GUI Window
	cv::imshow(OPENCV_WINDOW2, result_dil);
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}// end ImageConverter::imageCb



std::vector<cv::Point2f> ImageConverter::pvt_identify_pt(std::vector<cv::Point2f> pts_to_identify, std::vector<cv::Point2f> old_vec)
{

	// This is a vector of vectors (aka a matrix), 4x4 in most cases, and it will house the distance from each point to the other.
	float rtn_vec[NUMPTS][NUMPTS];
	
	for(int i=0; i<pts_to_identify.size();i++) 
	{
		//We use -1 from the old vector size because the mean is included in old vector
		for(int j = 0; j < (old_vec.size()-1); j++)
			{
			// Calculate distance from each old point (except the last one, which corresponds to the mean
			rtn_vec[i][j] = distance_calc(old_vec[j],pts_to_identify[i]);
		}// end of internal nested for
	} // end of external nested for


	// We now have a 4x4 matrix populated by the distances to nearby points.
	// We essentially want to select the lowest sum possible that still incorporates all distances.
	// The code below calculates all possible combinations for the point movements, and will select the minimum one.
	std::vector<float> sum_dist_vec(24);
	int min_dist_ind = 0;
	int index = 0;
	int order[NUMPTS];
	
	
for (int i = 0; i < NUMPTS; i++)
{
	for(int j = 0; j < NUMPTS; j++)
	{
		if (i!=j)
		{
			for(int k=0; k <NUMPTS; k++)
			{
				if (i!=k && j!=k)
				{
					for (int m=0; m<NUMPTS; m++)
					{
						if (i!=m && j!=m && k!=m)
						{
							//std::cout << "i : " << i << " j : " << j << " k : " << k << " m : " << m;
							sum_dist_vec[index] = rtn_vec[0][i] + rtn_vec[1][j] + rtn_vec[2][k] + rtn_vec[3][m];
							if (sum_dist_vec[index] < sum_dist_vec[min_dist_ind]) 
							{
								min_dist_ind = index;
								order[0] = i;
								order[1] = j;
								order[2] = k;
								order[3] = m;
							}
							//std::cout << "minimum index is: " << min_dist_ind;
							index++;
						}
					}
				}
			}
		}//endif 
	}//endfor
}

std::vector<cv::Point2f> test;
test.resize(4);


for (int p = 0; p < NUMPTS; p++)
{
	test[order[p]] = pts_to_identify[p];
}

	return test;
} // end pvt_identify_pt



float distance_calc(cv::Point2f pt1, cv::Point2f pt2)
{
	float result;
	
	result = sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x)+(pt2.y-pt1.y)*(pt2.y-pt1.y));
	return result;
} // End distance_calc


//This function is simply meant to help with the wraparound of the whole 
int within_bounds(int index)
{

	if (index >= 2*NUMPTS)
	{
		ROS_ERROR("Index is twice what it should be");
	}
// If the index is greater than numpoints, that means that we want to be wrapping around the matrix
	else if (index >= NUMPTS)
	{
		return (index-NUMPTS);
	}
	else {return index;}
}

