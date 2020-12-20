#include <aeplanner/planner.h>



void test_planner(OcTree& tree){
	cout << "============Test Planner==============" << endl;
	Node* start = randomConfig(tree);
	int max_sample = 200;
	int num_sample = 100;
	double eps = 0.5;
	KDTree* t = new KDTree();
	double best_IG = 0;
	std::vector<geometry_msgs::Point> tree_vis_array;
	KDTree* cache = new KDTree();
	std::vector<Node*> best_branch = planner(tree, start, num_sample, max_sample, eps, best_IG, tree_vis_array, t, cache);
	print_node_vector(best_branch);
	// cout << "RRT vis array size: " << tree_vis_array.size() << endl;
	// t->clear();
	// for (int i=1; i<best_branch.size(); ++i){
	// 	if (i == best_branch.size()-1){
	// 		t->setBest(best_branch[i]);
	// 	}
	// 	if (i == 1){
	// 		best_branch[i]->parent = NULL;
	// 		best_branch[i]->ig = 0; 
	// 	}
	// 	else{
	// 		best_branch[i]->ig = best_branch[i-1]->ig + exp(-lambda*eps) * calculateUnknown(tree, best_branch[i]);
	// 	}
	// }
	// tree_vis_array.clear();
	// best_branch = planner(tree, start, num_sample, max_sample, eps, best_IG, tree_vis_array,t, cache);
	// print_node_vector(best_branch);
	cout << "RRT vis array size: " << tree_vis_array.size() << endl;
	cout << "======================================" << endl;
	return;
}





int main(int argc, char** argv){
	cout << "Test Planner" << endl;
	OcTree tree (0.1);
	tree.readBinary("/home/zhefan/check_rrt.bt");
	tree.setResolution(0.2);
	cout << "Resolution of the map is: " << tree.getResolution() << endl;
	// Test Planner
	test_planner(tree);
	return 0;
}