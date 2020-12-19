#include <iostream>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <algorithm>
#include <stdlib.h>
using namespace octomap;
using std::cout; using std::endl;

// =====================RRT/KDTree NODE=================================
typedef struct Node{
	point3d p; // (x, y, z) see octomath::vector3
	double yaw; // yaw angle rads
	double ig;  // Information Gain
	double sc_gain; // shortcut gain
	double num_unknown; // number of unknown voxels
	struct Node* tree_parent; // kd-tree parent
	double dis;
	struct Node* parent; // Parent Node For RRT Backtracking
	struct Node* left;   // kd-tree left
	struct Node* right;  // kd-tree right
	Node(point3d _p, double _yaw){
		p = _p;
		yaw = _yaw;
		ig = 0;
		dis = 0;
		parent = NULL;
		left = NULL;
		right = NULL;
	}
} Node;

//======================================================================


//===================Class Declaration==================================
class KDTree{
private:
	int size;
	Node* root;
	std::vector<Node*> not_target;
	Node* best; // Highest info gain
	Node* best_sc; // Best Node for shortcut
public:
	KDTree();
	Node* getRoot();
	int getSize();
	void insert(Node* n);
	Node* nearestNeighbor(Node* n, 
					      Node* root_node,
					      Node* best_node,
						  double least_distance,
						  int depth); // return pointer to the nearest neighbor
	void setBest(Node* n);
	Node* getBest();
	void setBestSc(Node* n);
	Node* getBestSc();
	void clear();   // empty tree
};

//======================================================================


// ===================Implementation====================================
KDTree::KDTree(){
	// root = NULL;
	size = 0;
}

Node* KDTree::getRoot(){
	return root;
}

int KDTree::getSize(){
	return size;
}

void KDTree::insert(Node* n){
	n->left = NULL;
	n->right = NULL;
	// If tree is emtpy, we add root
	if (size == 0){
		root = n;
		++size;
		return;
	}
	else{
		Node* ptr = root;
		int depth = 0;
		double value, insert_value;
		while (true){
			if (depth % 3 == 0){
				value = ptr->p.x();
				insert_value = n->p.x();
			}
			else if (depth % 3 == 1){
				value = ptr->p.y();
				insert_value = n->p.y();

			}
			else if (depth % 3 == 2){
				value = ptr->p.z();
				insert_value = n->p.z();

			}
			// Compare the value:
			// >=: right, < left
			if (insert_value >= value){
				if (ptr->right == NULL){
					ptr->right = n;
					n->tree_parent = ptr;
					++size;
					return;
				}
				ptr = ptr->right;
			}
			else{
				if (ptr->left == NULL){
					ptr->left = n;
					n->tree_parent = ptr;
					++size;
					return;
				}
				ptr = ptr->left;
			}
			++depth;
		}
	}
	return;
}

static double least_distance_nn = 10000000;
Node* KDTree::nearestNeighbor(Node* n, 
							  Node* root_node=NULL,
							  Node* best_node = NULL,
							  double least_distance=1000000,
							  int depth=0){
	point3d p = n->p; // get position/coordinate of the node
	Node* ptr;
	if (root_node == NULL){
		ptr = root;	
	}
	else{
		ptr = root_node;
	}
	// Search Good side
	// Store Bad side
	std::vector<Node*> bad_side;
	while (ptr != NULL){
		
		// cout << "depth: " << depth << endl;
	 //    cout << "point: "<< ptr->p.x() <<", " <<  ptr->p.y()<<", " << ptr->p.z() << endl;

		// Check current node againt the best node
		double distance_current = p.distance(ptr->p);

		//==============Nodes Which are not our target================
 		if (ptr == n){
			distance_current = 10000000;
		}
		for (Node* nt: this->not_target){
			if (ptr == nt){
				distance_current = 10000000;
				break;
			}
		}

		//============================================================
		if (distance_current < least_distance_nn){	
			best_node = ptr;
			least_distance_nn = distance_current;
		}


		// Determine which side is better:
		double value, query_value;
		if (depth % 3 == 0){
			value = ptr->p.x();
			query_value = p.x();
		}
		else if (depth % 3 == 1){
			value = ptr->p.y();
			query_value = p.y();
		}
		else if (depth % 3 == 2){
			value = ptr->p.z();
			query_value = p.z();
		}

		// if < then search left, if >= search right
		if (query_value < value){
			bad_side.push_back(ptr->right);	
			ptr = ptr->left;
			// cout << "left" << endl;
		}
		else{
			bad_side.push_back(ptr->left);	
			ptr = ptr->right;
			// cout << "right" << endl;
		}
		++depth;
	}

	// Search bad side:
	// cout << depth << endl;
	std::reverse(bad_side.begin(), bad_side.end());
	// cout << "bad side size :" << bad_side.size() << endl;
	// cout << "bad side size: " << bad_side.size() << endl;
	// cout << "least distance after good search: " << least_distance << endl; 
	int count = 0;
	for (std::vector<Node*>::iterator itr = bad_side.begin(); 
		 itr != bad_side.end(); ++itr){
		// cout << "count: " << count << endl;
		++count;
		// If there is no node in bad side
		if (*itr == NULL){
			// cout << "no branch" << endl;
			// cout << "depth NULL: " << depth << endl;
			--depth;
			continue;
		}
		else{
			// cout << "have branch" << endl;
			double value, query_value;
			if ((depth-1) % 3 == 0){
				value = (*itr)->tree_parent->p.x();
				query_value = p.x();
			}
			else if ((depth-1) % 3 == 1){
				value = (*itr)->tree_parent->p.y();
				query_value = p.y();
			}
			else if ((depth-1) % 3 == 2){
				value = (*itr)->tree_parent->p.z();
				query_value = p.z();
			}
			double best_bad_side_distance = std::abs(value - query_value);
			// cout << "value, query_value: " << value << " " << query_value << endl;
			// cout << "depth: " << depth << "  best bad: " << best_bad_side_distance << endl;
			// if (best_node != NULL){
			// 	cout << "current best: "<< best_node->p.x() <<", " <<  best_node->p.y()<<", " << best_node->p.z() << endl;
			// }
			// else{
			// 	cout << "current best is null" << endl;
			// }
			// cout << "least_distance: " << least_distance << endl;
			if (best_bad_side_distance >= least_distance){

				// cout << "best distance is not good enough" << endl;
				// cout << "best bad side distance: " << best_bad_side_distance << endl;
				// cout << "least distance: " << least_distance << endl;
				--depth;
				continue;
			}
			else{
				// cout << "recursive call at depth : " << depth << endl;
				// cout << "root point: "<< (*itr)->p.x() <<", " <<  (*itr)->p.y()<<", " << (*itr)->p.z() << endl;
				// cout << "least_distance: " << least_distance << endl;
				best_node = nearestNeighbor(n, *itr, best_node, least_distance, depth);
				--depth;	
			}
		}	
	}

	if (root_node == NULL){
		least_distance_nn = 1000000;
	}
	return best_node;
}

void KDTree::setBest(Node* n){
	best = n;
}

Node* KDTree::getBest(){
	return best;
}

void KDTree::setBestSc(Node* n){
	best_sc = n;
}

Node* KDTree::getBestSc(){
	return best_sc;
}

void KDTree::clear(){
	root = NULL;
	size = 0;
	return;
}