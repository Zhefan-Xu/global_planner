/*
*	File: dep_node.cpp
*	---------------
*   dynamic exploration planner test
*/

#include <global_planner/dep.h>

int main(int argc, char**argv){
	ros::init(argc, argv, "dep_test_node");
	ros::NodeHandle nh;
	

	std::shared_ptr<mapManager::dynamicMap> m;
	m.reset(new mapManager::dynamicMap ());
	m->initMap(nh);

	globalPlanner::DEP p (nh);
	p.initMap(m);

	p.makePlan();
	ros::spin();

	return 0;
}