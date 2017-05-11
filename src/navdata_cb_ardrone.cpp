#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "navdata_cb_ardrone.h"

navdata_cb_ardrone::navdata_cb_ardrone()
{
    std::cout << "in the constructor:";
    // Subscrive to input video feed and publish output video feed
    navdata_sub_ = nh_.subscribe("/ardrone/navdata", 1, &navdata_cb_ardrone::callback, this);

}

void navdata_cb_ardrone::callback(const ardrone_autonomy::Navdata& msg)
{
	roll = msg.rotX;
	pitch = msg.rotY;
	yaw = msg.rotZ;

    get_rpy();
}


std::vector<double> navdata_cb_ardrone::get_rpy()
{
	std::vector<double> output;
	output.push_back(roll);
	output.push_back(pitch);
	output.push_back(yaw);
	
    for(int i = 0; i<output.size();i++)
    {
        std::cout << output[i] << " ";
    }
    std::cout << '\n';

    return output;
}

void navdata_cb_ardrone::set_rpy(double newroll, double newpitch, double newyaw)
{
    roll = newroll;
    pitch = newpitch;
    yaw = newyaw;
}




