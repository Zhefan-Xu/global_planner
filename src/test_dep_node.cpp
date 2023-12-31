/*
*	File: test_dep_node.cpp
*	---------------
*   dynamic exploration planner test
*/

#include <global_planner/dep.h>

std::shared_ptr<mapManager::dynamicMap> m;
std::shared_ptr<globalPlanner::DEP> p;
bool replan = true;

// void replanCB(const ros::TimerEvent&){
// 	if (replan){
// 		cout << "start planning." << endl;
// 		ros::Time startTime = ros::Time::now();
// 		bool replanSuccess = p->makePlan();
// 		ros::Time endTime = ros::Time::now();
// 		cout << "planning time: " << (endTime - startTime).toSec() << "s." << endl;
// 		cout << "end planning." << endl;
// 		if (replanSuccess){
// 			replan = false; // only plan one time for test purpose.
// 		}
// 	}
// }

void replanDEP(){
	while (ros::ok()){
		// set the current map
		p->setMap(m);
		cout << "start planning." << endl;
		ros::Time startTime = ros::Time::now();
		p->makePlan();
		ros::Time endTime = ros::Time::now();
		cout << "planning time: " << (endTime - startTime).toSec() << "s." << endl;
		cout << "end planning." << endl;


		cout << "PRESS ENTER to Replan." << endl;
		std::cin.clear();
		fflush(stdin);
		std::cin.get();		
		// replan = true;	
	}


}


int main(int argc, char**argv){
	ros::init(argc, argv, "dep_test_node");
	ros::NodeHandle nh;
	

	m.reset(new mapManager::dynamicMap ());
	m->initMap(nh);

	// wait for some time
	ros::Rate r (50);
	ros::Time startTime = ros::Time::now();
	ros::Time currTime = ros::Time::now();
	while (ros::ok()){
		currTime = ros::Time::now();
		if ((currTime - startTime).toSec() >= 2.0){
			break;
		}
		ros::spinOnce();
		r.sleep();
	}

	p.reset(new globalPlanner::DEP (nh));
	p->setMap(m);

	std::thread planningWorker = std::thread(replanDEP);


	// ros::Timer replanTimer = nh.createTimer(ros::Duration(0.1), replanCB);	
	ros::spin();

	return 0;
}