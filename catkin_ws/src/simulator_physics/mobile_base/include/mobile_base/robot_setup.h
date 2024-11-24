float robot_pose_x;
float robot_pose_y;
float robot_pose_w;

bool setInitialPose() {
    if(ros::param::has("/robot_pose_x")) ros::param::get("robot_pose_x", robot_pose_x);
    else { ROS_ERROR("There's no parameter for robot_pose_x"); return false; }

    if(ros::param::has("/robot_pose_y")) ros::param::get("robot_pose_y", robot_pose_y);
    else { ROS_ERROR("There's no parameter for robot_pose_y"); return false; }

    if(ros::param::has("/robot_pose_w")) ros::param::get("robot_pose_w", robot_pose_w);
    else { ROS_ERROR("There's no parameter for robot_pose_w"); return false; }

    return true;
}
