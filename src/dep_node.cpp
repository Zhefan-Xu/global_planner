/*
*	File: dep_node.cpp
*	---------------
*   dynamic exploration planner test
*/

#include <global_planner/dep.h>

std::shared_ptr<mapManager::dynamicMap> m;
std::shared_ptr<globalPlanner::DEP> p;
bool replan = true;

void replanCB(const ros::TimerEvent&){
	if (replan){
		cout << "start planning." << endl;
		bool replanSuccess = p->makePlan();
		cout << "end planning." << endl;
		if (replanSuccess){
			replan = false; // only plan one time for test purpose.
		}
	}
}


int main(int argc, char**argv){
	ros::init(argc, argv, "dep_test_node");
	ros::NodeHandle nh;
	

	m.reset(new mapManager::dynamicMap ());
	m->initMap(nh);

	p.reset(new globalPlanner::DEP (nh));
	p->setMap(m);

	ros::Timer replanTimer = nh.createTimer(ros::Duration(0.1), replanCB);	
	ros::spin();

	return 0;
}