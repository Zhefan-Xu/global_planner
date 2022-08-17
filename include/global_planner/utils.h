/*
	FILE: utils.h
	--------------
	miscs
*/
#ifndef GLOBALPLANNER_UTILS_H
#define GLOBALPLANNER_UTILS_H
#include <random>

namespace globalPlanner{

	// Helper Function: Random Number
	inline double randomNumber(double min, double max){
		std::random_device rd;
		std::mt19937 mt(rd());
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(mt);
	}
}

#endif
