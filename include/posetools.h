#ifndef _POSETOOLS_H
#define _POSETOOLS_H

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

std::vector<Point2f> virtcam(std::vector<Point2f> input, double roll, double pitch)
{
//NEED TO ADD ACTUAL PROCESSING
    std::vector<Point2f> output(input.size());

    for(int i = 0 ; i < input.size(); i++)
    {
        output[i].x = input[i].x * roll;
        output[i].y = input[i].y * pitch;
    }
    return output;
}


std::vector<double> quat_to_rpy(const geometry_msgs::Quaternion msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    std::vector<double> output(3);
    output[0] = roll;
    output[1] = pitch;
    output[2] = yaw;
    return output;
}



// This function accepts the Left handed OptiTrack quaternion and cartesian coordinate system and outputs in a normal RHS orientation
geometry_msgs::PoseStamped Opti_Rect_pose(const geometry_msgs::PoseStamped& input)
  {
    // Create the PoseStamped output message
    geometry_msgs::PoseStamped output;
    // Maintain the same header
    output.header.seq= input.header.seq;
    // Re-map the x->x, y->z, z->-y (from input to output)...x->x, y->-z, z->y (from output to input)
    output.pose.position.x = input.pose.position.x;
    output.pose.position.y = -input.pose.position.z;
    output.pose.position.z = input.pose.position.y;
    
    // This is necessary because the OptiTrack appears to internally use a left-handed coordinate system.
    // The switching and inversion of the y and z components of the output pose appear to fix this.
    output.pose.orientation.x = input.pose.orientation.x;
    output.pose.orientation.y = -input.pose.orientation.z;
    output.pose.orientation.z = input.pose.orientation.y;
    output.pose.orientation.w = input.pose.orientation.w;

    return output;
}//End of function Opti_Rect

#endif
