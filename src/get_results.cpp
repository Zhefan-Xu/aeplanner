#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>




void callback(const octomap_msgs::Octomap::ConstPtr& bmap){
	abtree = octomap_msgs::binaryMsgToMap(*bmap);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "get_results");
	ros::NodeHandle nh;
	ros::Subscriber sub = n.subscribe("octomap_binary", 100, callback);
	ros::spin();
	return 0;
}