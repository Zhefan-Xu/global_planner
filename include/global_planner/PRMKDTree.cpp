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

	void KDTree::clear(){
		this->root_ = NULL;
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
			while (true){
				int index = int(depth % 3);
				double value = ptr->pos(index);
				double insertValue = n->pos(index);

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

	std::shared_ptr<Node> KDTree::nearestNeighbor(std::shared_ptr<Node> n,
	                             				  std::shared_ptr<Node> rootNode, 
	                             				  std::shared_ptr<Node> bestNode,
	                             				  int depth){
		std::shared_ptr<Node> ptr;
		if (rootNode == NULL){
			ptr = this->root_;
		}
		else{
			ptr = rootNode;
		}

		// Search good side and store bad side
		std::vector<std::shared_ptr<Node>> badSide;
		while (ptr != NULL){
			double currDist = (n->pos - ptr->pos).norm(); 

			// avoid finding the same node
	 		if (ptr == n){
				currDist = std::numeric_limits<double>::infinity();
			}
			// avoid finding node in this list
			// for (std::shared_ptr<Node> nt: this->notTarget_){
			// 	if (ptr == nt){
			// 		currDist = std::numeric_limits<double>::infinity();
			// 		break;
			// 	}
			// }

			// avoid target here
			if (this->notTargetTemp_.find(ptr) != this->notTargetTemp_.end()){
				currDist = std::numeric_limits<double>::infinity();
			}

			if (this->notTargetPerm_.find(ptr) != this->notTargetPerm_.end()){
				currDist = std::numeric_limits<double>::infinity();
			}

			if (currDist < this->leastDistNN_){
				bestNode = ptr;
				this->leastDistNN_ = currDist;
			}

			int index = int(depth % 3);
			double value = ptr->pos(index);
			double queryValue = n->pos(index);

			// if less than, bad side is the right side. Otherwise left side.
			if (queryValue < value){
				badSide.push_back(ptr->right);
				ptr = ptr->left;
			}
			else{
				badSide.push_back(ptr->left);
				ptr = ptr->right;
			}
			++depth;
		}


		// Search the previous bad side (from the latest to oldest)
		for (size_t i=0; i<badSide.size(); ++i){
			int searchIdx = int(badSide.size()) - i - 1;
			std::shared_ptr<Node> ptr = badSide[searchIdx];
			if (ptr == NULL){
				--depth;
				continue;
			}
			else{
				// recursively search the bad side's parent node
				int index = int((depth-1) % 3);
				double value = ptr->treeParent->pos(index);
				double queryValue = n->pos(index);

				double bestPossibleDist = std::abs(value - queryValue);
				if (bestPossibleDist >= this->leastDistNN_){
					--depth;
					continue;
				}
				else{
					bestNode = this->nearestNeighbor(n, ptr, bestNode, depth);
					--depth;
				}
			}
		}

		if (rootNode == NULL){
			this->leastDistNN_ = std::numeric_limits<double>::infinity(); 
		}
		return bestNode;
	}
	
	// Returns the k-nearest neighbor in ascending order
	std::vector<std::shared_ptr<Node>> KDTree::kNearestNeighbor(std::shared_ptr<Node> n, int num){
		std::vector<std::shared_ptr<Node>> knn;
		num = std::min(this->size_-1, num);
		for (int i=0; i<num; ++i){
			std::shared_ptr<Node> nearestNeighborNode = nearestNeighbor(n);
			if (nearestNeighborNode == NULL){
				if (n != NULL){
					cout << "invalid node: " << n->pos << endl;
				}
				else{
					cout << "NULL node." << endl;
				}
				cout << "find null pointer at " << i << endl;
			}
			knn.push_back(nearestNeighborNode);
			// this->notTarget_.push_back(nearestNeighborNode);
			this->notTargetTemp_.insert(nearestNeighborNode);
		}
		// this->notTarget_.clear();
		this->notTargetTemp_.clear();
		return knn;
	}

	void KDTree::remove(std::shared_ptr<Node> n){
		this->notTargetPerm_.insert(n);
		--this->size_;
	}
}
