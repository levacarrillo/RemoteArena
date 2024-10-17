#include <ros/ros.h>
#include <mobile_base/MoveMinibot.h>

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    std::cout << "Distance->" << req.distance << std::endl;
    std::cout << "Theta->" << req.theta << std::endl;
    res.done = true;
    return res.done;
}

int main(int argc, char **argv) {
    std::cout << "Starting move_minibot_node_simul by Luis Nava..." << std::endl;
    ros::init(argc, argv, "move_minibot_node_simul");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    ros::ServiceServer moveService = nh.advertiseService("move_robot", moveCallback);

    ros::spin();
    return 1;
}