#include <tf2_ros/static_transform_broadcaster.h>

float robot_pose_x;
float robot_pose_y;
float robot_pose_w;


bool setInitialPose() {
    if(ros::param::has("/mobile_base/robot_pose_x")) ros::param::get("/mobile_base/robot_pose_x", robot_pose_x);
    else { ROS_ERROR("There's no parameter for /mobile_base/robot_pose_x"); return false; }

    if(ros::param::has("/mobile_base/robot_pose_y")) ros::param::get("/mobile_base/robot_pose_y", robot_pose_y);
    else { ROS_ERROR("There's no parameter for /mobile_base/robot_pose_y"); return false; }

    if(ros::param::has("/mobile_base/robot_pose_w")) ros::param::get("/mobile_base/robot_pose_w", robot_pose_w);
    else { ROS_ERROR("There's no parameter for /mobile_base/robot_pose_w"); return false; }

    return true;
}

geometry_msgs::TransformStamped getTFStamped(std::string parent_frame, std::string child_frame, float robot_x, float robot_y, float robot_w) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = robot_x;
    transformStamped.transform.translation.y = robot_y;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, robot_w);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    return transformStamped;
}