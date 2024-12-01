#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mobile_base/mobile_base_utils.h>
#include <mobile_base/OdomSetPoint.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>


double robotX = 0.0, robotY = 0.0, robotT = 0.0;
double vx = 0.0, vth = 0.0;

ros::Publisher pubOdom;


void publishOdom(nav_msgs::Odometry odom, double dt) {
    robotX += vx * cos(robotT) * dt;
    robotY += vx * sin(robotT) * dt;
    robotT += vth * dt;
    // std::cout << "mobile_base.->x: " << robotX << "\ty: " << robotY << "\tw: " << robotT << std::endl;

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(getTFStamped("odom", "base_link", robotX, robotY, robotT));

    tf2::Quaternion q;
    q.setRPY(0, 0, robotT);

    geometry_msgs::Quaternion odom_quad = tf2::toMsg(q);
    odom.header.stamp = ros::Time(0);
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x = robotX;
	odom.pose.pose.position.y = robotY;
	odom.pose.pose.position.z =    0.0;
	odom.pose.pose.orientation = odom_quad;
    pubOdom.publish(odom);
}

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr &msg) {
    // std::cout << "mobile_base.->linear x: " << msg->linear.x << std::endl;
    // std::cout << "mobile_base.->angular z: "<< msg->angular.z << std::endl;
    vx  = msg->linear.x;
    vth = msg->angular.z;
}

bool odomCallback(mobile_base::OdomSetPoint::Request &req, mobile_base::OdomSetPoint::Response &res) {
    // std::cout << "mobile_base.-> req.x: " << req.robot_x <<  "\treq.y: " << req.robot_y << "\treq.w: " << req.robot_w << std::endl;
    static tf2_ros::StaticTransformBroadcaster br;
    br.sendTransform(getTFStamped("map", "odom",  req.robot_x, req.robot_y, req.robot_w));

    robotX = 0.0;
    robotY = 0.0;
    robotT = 0.0;

    res.done = true;
    return res.done;
}

int main(int argc, char ** argv) {
    std::cout << "Starting mobile_base_node by Luis Nava..." << std::endl;
	ros::init(argc, argv, "mobile_base_node");
	ros::NodeHandle nh;
    ros::Rate rate(30);

    ros::Subscriber subCmdVel = nh.subscribe("/mobile_base/cmd_vel", 1, callbackCmdVel);
    // ros::Publisher pubJointState   = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::ServiceServer odomService = nh.advertiseService("/mobile_base/odom_set_point", odomCallback);
    
    pubOdom  = nh.advertise<nav_msgs::Odometry>("odom", 1);

    if (!setInitialPose()) { return -1; }

	// std::string jointNames[2] = {"left_wheel_joint_connect", "right_wheel_joint_connect"};
	// float jointPositions[2] = {0.0, 0.0};

	nav_msgs::Odometry odom;
	// sensor_msgs::JointState jointState;

	// jointState.name.insert(jointState.name.begin(), jointNames, jointNames + 2);
	// jointState.position.insert(jointState.position.begin(), jointPositions, jointPositions + 2);

    static tf2_ros::StaticTransformBroadcaster br;
    br.sendTransform(getTFStamped("map", "odom", robot_pose_x, robot_pose_y, robot_pose_w));
    
    while(ros::ok()) {
        double dt = rate.expectedCycleTime().toSec();

        publishOdom(odom, dt);
        // pubJointState.publish(jointState);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
