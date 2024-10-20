#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double robotX = 0.0, robotY = 0.0, robotT = 0.0;
double vx = 0.0, vth = 0.0;

ros::Publisher pubOdom;

void publishOdom(nav_msgs::Odometry odom, double dt) {
    robotX += vx * cos(robotT) * dt;
    robotY += vx * sin(robotT) * dt;
    robotT += vth * dt;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = robotX;
    transformStamped.transform.translation.y = robotY;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, robotT);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);

    geometry_msgs::Quaternion odom_quad = tf2::toMsg(q);
    odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x = robotX;
	odom.pose.pose.position.y = robotY;
	odom.pose.pose.position.z =    0.0;
	odom.pose.pose.orientation = odom_quad;
    pubOdom.publish(odom);
}

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr &msg) {
    // std::cout << "Received command robot velocity"<< std::endl;
    // std::cout << "linear x->" << msg->linear.x << std::endl;
    // std::cout << "angular z->"<< msg->angular.z << std::endl;

    vx  = msg->linear.x;
    vth = msg->angular.z;
}

int main(int argc, char ** argv) {
    std::cout << "Starting mobile_base_node_simul by Luis Nava..." << std::endl;
	ros::init(argc, argv, "mobile_base_node_simul");
	ros::NodeHandle nh;
    ros::Rate rate(60);

    ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel", 1, callbackCmdVel);
    ros::Publisher pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    pubOdom  = nh.advertise<nav_msgs::Odometry>("odom", 1);

	std::string jointNames[2] = {"left_wheel_joint_connect", "right_wheel_joint_connect"};
	float jointPositions[2] = {0.0, 0.0};

	sensor_msgs::JointState jointState;
	nav_msgs::Odometry odom;

	jointState.name.insert(jointState.name.begin(), jointNames, jointNames + 2);
	jointState.position.insert(jointState.position.begin(), jointPositions, jointPositions + 2);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "odom";
    static_transformStamped.transform.translation.x = 0.2;
    static_transformStamped.transform.translation.y = 0.5;
    static_transformStamped.transform.translation.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);

    while(ros::ok()) {
        double dt = rate.expectedCycleTime().toSec();

        publishOdom(odom, dt);
        pubJointState.publish(jointState);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
