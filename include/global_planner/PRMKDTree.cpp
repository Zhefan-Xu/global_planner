/*
*	File: PRMKDTree.cpp
*	---------------
*   PRM KDTree Implementation
*/

#include <global_planner/PRMKDTree.h>
namespace PRM{
	KDTree::KDTree(){
		this->size_ = 0;
	}

	std::shared_ptr<Node> KDTree::getRoot(){
		return this->root_;
	}

	int KDTree::getSize(){
		return this->size_;
	}

	void KDTree::insert(std::shared_ptr<Node> n){
		// set the newly inserted node child to NULL
		n->left = NULL;
		n->right = NULL;

		// if tree is empty, we add the node as root node
		if (this->size_ == 0){
			this->root_ = n;
			++this->size_;
			return;
		}
		else{
			std::shared_ptr<Node> ptr = this->root_;
			int depth = 0;
			double value, insertValue;
			while (true){
				if (depth % 3 == 0){
					value = ptr->pos(0);
					insertValue = n->pos(0);
				}
				else if (depth % 3 == 1){
					value = ptr->pos(1);
					insertValue = n->pos(1);
				}
				else if (depth % 3 == 2){
					value = ptr->pos(2);
					insertValue = n->pos(2);
				}

				if (insertValue >= value){
					if (ptr->right == NULL){
						ptr->right = n;
						n->treeParent = ptr;
						++this->size_;
						return;
					}
					ptr = ptr->right;
				}
				else{
					if (ptr->left == NULL){
						ptr->left = n;
						n->treeParent = ptr;
						++this->size_;
						return;
					}
					ptr = ptr->left;
				}
				++depth;
			}
		}
	}
}