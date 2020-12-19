#include <aeplanner/rrt.h>
#include <algorithm>


// HELPER FUNCTION: Extract Best Path
void extractBestBranch(KDTree* t, std::vector<Node*>& best_branch){
	// Get Best Node
	Node* best_node = (*t).getBest();
	Node* ptr = best_node;
	while (ptr != NULL){
		best_branch.push_back(ptr);
		ptr = ptr->parent;
	}
	std::reverse(best_branch.begin(), best_branch.end());
}

// HELPER FUNCTION: Extract Best Path
void extractBestBranchSc(KDTree* t, std::vector<Node*>& best_branch){
	// Get Best Node
	Node* best_node = (*t).getBestSc();
	Node* ptr = best_node;
	while (ptr != NULL){
		best_branch.push_back(ptr);
		ptr = ptr->parent;
	}
	std::reverse(best_branch.begin(), best_branch.end());
}



// Planner Function:
std::vector<Node*> planner(OcTree& tree, 
						  Node* start,
						  int num_sample,
						  int max_sample,
						  double eps,
						  double& best_IG,
						  std::vector<geometry_msgs::Point> &tree_vis_array,
						  KDTree* t,
						  bool write_result = false){
	// Intialize 
	std::vector<Node*> best_branch;
	// Grow RRT Tree
	KDTree* new_t = growRRT(tree, start, num_sample, max_sample, eps, best_IG,  tree_vis_array, t, write_result);
	// Extract the best branch
	if (new_t == NULL){
		return best_branch;
	}
	extractBestBranch(new_t, best_branch);
	// extractBestBranchSc(new_t, best_branch);

	return best_branch;
} 