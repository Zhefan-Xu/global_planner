/*
*	File: test_prm_kdtree.cpp
*	---------------
*   unit test for kdtree implementation
*/

#include <global_planner/PRMKDTree.h>

using std::cout; using std::endl;

void print_node(std::shared_ptr<PRM::Node> n){
	cout << "(" << n->pos(0) << ", " << n->pos(1) << ", " << n->pos(2) << ")" << endl;;
}



void test_kdtree_insert(){
	cout << "============Test KDTree Insert==============" << endl;
	Eigen::Vector3d p1 (1, 2, 3);
	Eigen::Vector3d p2 (2, 3, 4);
	Eigen::Vector3d p3 (3, 4, 5);
	Eigen::Vector3d p4 (5, 2, 1);
	Eigen::Vector3d p5 (10, 0, 10);
	std::shared_ptr<PRM::Node> n1 (new PRM::Node (p1));
	std::shared_ptr<PRM::Node> n2 (new PRM::Node (p2));
	std::shared_ptr<PRM::Node> n3 (new PRM::Node (p3));
	std::shared_ptr<PRM::Node> n4 (new PRM::Node (p4));
	std::shared_ptr<PRM::Node> n5 (new PRM::Node (p5));

	//Initiaize KDTree;
	std::shared_ptr<PRM::KDTree> t1 (new PRM::KDTree ); 
	t1->insert(n2);
	t1->insert(n1);
	t1->insert(n3);
	t1->insert(n4);
	t1->insert(n5);
	cout << "size of kdtree: " << t1->getSize() <<  endl;
	std::shared_ptr<PRM::Node> root = t1->getRoot();
	print_node(root);
	print_node(root->left);
	print_node(root->right);
	print_node(root->right->left);
	print_node(root->right->left->right);
	// print_node((*root));
	// print_node(*(root->left));
	// print_node(*(root->right));
	// print_node(*(root->right->left));
	cout << "============================================" << endl;
	return;
}


void test_kdtree_nn(){
	cout << "==========Test KDTree Nearest Neighbor=========" << endl;
	Eigen::Vector3d p1 (1, 2, 3);
	Eigen::Vector3d p2 (2, 3, 4);
	Eigen::Vector3d p3 (3, 4, 5);
	Eigen::Vector3d p4 (5, 2, 1);
	Eigen::Vector3d p5 (10, 0, 10);

	std::shared_ptr<PRM::Node> n1 (new PRM::Node (p1));
	std::shared_ptr<PRM::Node> n2 (new PRM::Node (p2));
	std::shared_ptr<PRM::Node> n3 (new PRM::Node (p3));
	std::shared_ptr<PRM::Node> n4 (new PRM::Node (p4));
	std::shared_ptr<PRM::Node> n5 (new PRM::Node (p5));


	//Initiaize KDTree;
	//Initiaize KDTree;
	std::shared_ptr<PRM::KDTree> t1 (new PRM::KDTree ); 
	t1->insert(n2);
	t1->insert(n1);
	t1->insert(n3);
	t1->insert(n4);
	t1->insert(n5);
	// cout << "size of kdtree: " << t1.getSize() <<  endl;

	// Find nearest neighbor for n6
	Eigen::Vector3d p6 (9, 0, 9); // nearest neighbor should be (10, 0, 10)
	std::shared_ptr<PRM::Node> n6 (new PRM::Node (p6));
	std::shared_ptr<PRM::Node> nn1 = t1->nearestNeighbor(n6);
	print_node(nn1);

	cout << "=====================" << endl;

	// Find nearest neighbor for n7
	Eigen::Vector3d p7 (2.1, 2, 3); // nearest neighbor should be (1, 2, 3)
	std::shared_ptr<PRM::Node> n7 (new PRM::Node (p7));
	std::shared_ptr<PRM::Node> nn2 = t1->nearestNeighbor(n7);
	print_node(nn2);
	cout << "=====================" << endl;

	// Find nearest neighbor for n9
	Eigen::Vector3d p8 (6, 3, 4);
	Eigen::Vector3d pa (6, 1, 0);
	Eigen::Vector3d pb (3, 3, 0);
	Eigen::Vector3d pc (9, 3, 0);
	Eigen::Vector3d p9 (9, 3, 2); // nearest neighbor should be (9, 3, 0)
	std::shared_ptr<PRM::Node> n8 (new PRM::Node (p8));
	std::shared_ptr<PRM::Node> na (new PRM::Node (pa));
	std::shared_ptr<PRM::Node> nb (new PRM::Node (pb));
	std::shared_ptr<PRM::Node> nc (new PRM::Node (pc));
	std::shared_ptr<PRM::Node> n9 (new PRM::Node (p9));
	t1->insert(na);
	t1->insert(nb);
	t1->insert(nc);
	t1->insert(n8);
	std::shared_ptr<PRM::Node> nn3 = t1->nearestNeighbor(n9);
	print_node(nn3);

	// Test Clear Tree:
	t1->clear();
	cout << "size of tree after clear: " << t1->getSize() << endl;
	cout << "root: " << t1->getRoot() << endl;
	cout << "===============================================" << endl;
}

int main(int argc, char** argv){
	test_kdtree_insert();
	test_kdtree_nn();

	return 0;
}