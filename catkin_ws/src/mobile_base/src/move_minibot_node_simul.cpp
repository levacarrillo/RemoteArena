#include <ros/ros.h>
#include <mobile_base/MoveMinibot.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

struct {
  double x;
  double y;
  double th;
} init, goal, curr;

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    std::cout << "Distance->" << req.distance << std::endl;
    std::cout << "Theta->" << req.theta << std::endl;

    init.x  = 0;
    init.y  = 0;
    init.th = 0;
    res.done = true;
    return res.done;
}

int main(int argc, char **argv) {
    std::cout << "Starting move_minibot_node_simul by Luis Nava..." << std::endl;
    ros::init(argc, argv, "move_minibot_node_simul");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    ros::ServiceServer moveService = nh.advertiseService("move_robot", moveCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;
        
        try {
	    transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
	    std::cout << "--- PARAMETERS ---" << std::endl;
	    std::cout << "x->" << transformStamped.transform.translation.x << std::endl;
	    std::cout << "y->" << transformStamped.transform.translation.y << std::endl;
	    std::cout << "th->" << transformStamped.transform.rotation.z << std::endl;

	} catch (tf2::TransformException &ex) {
	    ROS_WARN("%s", ex.what());
	    ros::Duration(1.0).sleep();
	    continue;
	}	
        
	rate.sleep(); 
    }

    return 1;
}
