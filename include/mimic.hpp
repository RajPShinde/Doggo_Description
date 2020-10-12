#ifndef INCLUDE_MIMIC_HPP_
#define INCLUDE_MIMIC_HPP_

#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

class Mimic
{
    public:

        ros::Publisher leg13Pub;
        ros::Publisher leg14Pub;
        ros::Publisher leg23Pub;
        ros::Publisher leg24Pub;
        ros::Publisher leg33Pub;
        ros::Publisher leg34Pub;
        ros::Publisher leg43Pub;
        ros::Publisher leg44Pub;

        double leg11, leg12, leg21, leg22, leg31, leg32, leg41, leg42;

    	Mimic(ros::NodeHandle& nh);

    	~Mimic();

    	void leg11Callback(std_msgs::Float64 msg);

    	void leg12Callback(std_msgs::Float64 msg);

    	void leg21Callback(std_msgs::Float64 msg);

    	void leg22Callback(std_msgs::Float64 msg);

    	void leg31Callback(std_msgs::Float64 msg);

    	void leg32Callback(std_msgs::Float64 msg);

    	void leg41Callback(std_msgs::Float64 msg);

    	void leg42Callback(std_msgs::Float64 msg);

        std_msgs::Float64 calculateBeta(double, double, int);

    private:
        double link1_, link2_;
        std_msgs::Float64 initialAngle_;
        double PI_ = 3.14159265;;


        ros::NodeHandle mimic_;
        ros::Subscriber leg11Sub;
        ros::Subscriber leg12Sub;
        ros::Subscriber leg21Sub;
        ros::Subscriber leg22Sub;
        ros::Subscriber leg31Sub;
        ros::Subscriber leg32Sub;
        ros::Subscriber leg41Sub;
        ros::Subscriber leg42Sub;

};

#endif  //  INCLUDE_MIMIC_HPP_
