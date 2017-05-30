#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "navdata_cb_ardrone.h"
#include <cmath>

navdata_cb_ardrone::navdata_cb_ardrone()
{
    std::cout << "in the constructor:";
    // Subscrive to input video feed and publish output video feed
    navdata_sub_ = nh_.subscribe("/ardrone/navdata", 1, &navdata_cb_ardrone::callback, this);

    count =0;
}

void navdata_cb_ardrone::callback(const ardrone_autonomy::Navdata& msg)
{

    roll = msg.rotX*M_PI/180;
    pitch = msg.rotY*M_PI/180;
    yaw = msg.rotZ*M_PI/180;

    std::cout << "callback roll, pitch, yaw: " << roll << '\t' << pitch << '\t' << yaw << '\n';
    std::vector<double> abc;
    abc.resize(3);
    abc = get_rpy();
    //getRotM();
}


std::vector<double> navdata_cb_ardrone::get_rpy()
{
    std::vector<double> output(3);

/*    output.push_back(roll);
	output.push_back(pitch);
	output.push_back(yaw);

    std::cout << "\n Inside get_rpy: \n";
    if(count%1 ==0)
    {
        for(int i = 0; i<output.size();i++)
        {
            std::cout << output[i] << " ";
        }
        std::cout << '\n';
    }
*/
    count+=1;


    std::cout << "\n\ngetrpy: " << roll << " " << pitch << " " << yaw<< " " << count << "\n";
    output.resize(3);
    return output;
}

Eigen::Matrix3d navdata_cb_ardrone::getRotM()
{
    Eigen::Matrix3d output;
    get_rpy();

    // Used number 2 of the rotation matrices from May 29th 2017 log
    output(0,0) = 1;
    output(1,0) = 0;
    output(0,1) = roll*pitch;
    output(1,1) = 1;
    output(2,0) = -pitch;
    output(2,1) = roll;
    output(2,2) = 1;
    output(0,2) = pitch;
    output(1,2) = -roll;

    std::cout << '\n' << "In Navdata cb\n";

    std::cout << output(0,0) << " " << output(0,1) << " " << output(0,2) << "\n" <<\
                 output(1,0) << " " << output(1,1) << " " << output(1,2) << "\n" <<\
                 output(2,0) << " " << output(2,1) << " " << output(2,2) << "\n";

    return output;
}

void navdata_cb_ardrone::set_rpy(double newroll, double newpitch, double newyaw)
{
    roll = newroll;
    pitch = newpitch;
    yaw = newyaw;
}


