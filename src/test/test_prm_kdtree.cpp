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

int main(int argc, char** argv){
	test_kdtree_insert();


	return 0;
}