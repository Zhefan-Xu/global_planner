/*
	FILE: utils.h
	--------------
	miscs
*/
#ifndef GLOBALPLANNER_UTILS_H
#define GLOBALPLANNER_UTILS_H
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

namespace globalPlanner{
	const double PI_const = 3.1415926;
	// Helper Function: Random Number
	inline double randomNumber(double min, double max){
		std::random_device rd;
		std::mt19937 mt(rd());
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(mt);
	}

    inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
    	if (yaw > PI_const){
    		yaw = yaw - 2*PI_const;
    	}
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
    	// return is [0, 2pi]
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	double roll, pitch, yaw;
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    	return yaw;
    }

    inline void rpy_from_quaternion(const geometry_msgs::Quaternion& quat, double &roll, double &pitch, double &yaw){
    	tf2::Quaternion tf_quat;
    	tf2::convert(quat, tf_quat);
    	tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }
}

#endif
