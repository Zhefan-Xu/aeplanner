#include <ros/ros.h>
#include <aeplanner/rrt.h>


void test_check_validation(const OcTree& tree){

}

void test_random_config(const OcTree& tree){
	cout << "============Test Random Configuration========" << endl;
	Node* nptr = randomConfig(tree);
	print_node(*nptr);
	cout << "=============================================" << endl;
}

void test_calUnknown(const OcTree& tree){
	cout << "================Test Count Unknown==============" << endl;
	Node* nptr = randomConfig(tree);
	int num = calculateUnknown(tree, nptr);
	cout << "Number of Unknown is: " << num << endl;
	cout << "================================================" << endl;
}

void test_rrt(OcTree& tree){
	cout << "============Test Grow Tree=========" << endl;
	Node* start = randomConfig(tree);
	double best_IG = 0;
	std::vector<geometry_msgs::Point> tree_vis_array;
	// KDTree* t = growRRT(tree, start, 100, 200,0.5,  best_IG, tree_vis_array);
	KDTree* t_ = new KDTree();
	KDTree* cache = new KDTree();
	KDTree* t = growRRT(tree, start, 500, 1000,0.5,  best_IG, tree_vis_array, t_, cache, true);
	Node* start2 = randomConfig(tree);
	double best_IG2 = 0;
	std::vector<geometry_msgs::Point> tree_vis_array2;
	KDTree* t2_ = new KDTree();
	KDTree* cache2 = new KDTree();
	KDTree* t2 = growRRT(tree, start2, 500, 1000,0.5,  best_IG2, tree_vis_array2, t2_, cache, true);
	cout << "Finish" << endl;
	cout << "===================================" << endl;
}

int main(int argc, char** argv){
	cout << "Test RRT" << endl;
	OcTree tree (0.1);
	tree.readBinary("/home/zhefan/check_rrt.bt");
	cout << "Resolution of the map is: " << tree.getResolution() << endl;
	double min_x, max_x, min_y, max_y, min_z, max_z;
	tree.getMetricMax(max_x, max_y, max_z);
	tree.getMetricMin(min_x, min_y, min_z);
	cout << "x range: " << min_x << "~" << max_x << endl;
	cout << "y range: " << min_y << "~" << max_y << endl;
	cout << "z range: " << min_z << "~" << max_z << endl;


	// test random config:
	// test_random_config(tree);

	// test grow rrt tree:
	test_rrt(tree);

	// test calculate unknown
	// test_calUnknown(tree);
	return 0;
}