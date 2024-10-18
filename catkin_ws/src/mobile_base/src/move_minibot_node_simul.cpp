#include <ros/ros.h>
#include <mobile_base/MoveMinibot.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

struct robotPose {
  double x;
  double y;
  double th;
} init, goal, curr, error;

enum State {
    SM_MOVE_ROBOT,
    SM_CORRECT_ANGLE,
    SM_FINISH_MOVEMENT
};

float ANGLE_TOLERANCY = 0.02;
float DISTANCE_TOLERANCY = 0.04;
ros::Publisher pubCmdVel;

robotPose getCurrentPose(tf2_ros::Buffer& tfBuffer) {
    robotPose currentPose;
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));

        currentPose.x = transformStamped.transform.translation.x;
        currentPose.y = transformStamped.transform.translation.y;
        currentPose.th = transformStamped.transform.rotation.z;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return currentPose;
}

robotPose getGoalPose(mobile_base::MoveMinibot::Request req) {
    robotPose goalPose;
    goalPose.x = init.x + req.distance * cos(init.th + req.theta);
    goalPose.y = init.y + req.distance * sin(init.th + req.theta);
    goalPose.th = init.th + req.theta;

    return goalPose;
}

robotPose getErrorPose() {
    robotPose errorPose;
    errorPose.x = goal.x - curr.x;
    errorPose.y = goal.y - curr.y;
    errorPose.th = goal.th - curr.th;
    return errorPose;
}

geometry_msgs::Twist getAngularMovement() {
    geometry_msgs::Twist movement;
    movement.linear.x = 0.0;
    movement.linear.y = 0.0;
    movement.angular.z = 0.1;

    return movement;
}

geometry_msgs::Twist getLinearMovement() {
    geometry_msgs::Twist movement;
    movement.linear.x = 0.1;
    movement.linear.y = 0.0;
    movement.angular.z = 0.0;

    return movement;
}

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    
    init = getCurrentPose(tfBuffer);
    goal = getGoalPose(req);

    geometry_msgs::Twist stop;
    stop.linear.x = 0.0;
    stop.linear.y = 0.0;
    stop.angular.z = 0.0;

    State state = SM_CORRECT_ANGLE;

    while(ros::ok() && !res.done) {
        
        curr = getCurrentPose(tfBuffer);
        error = getErrorPose();
        std::cout << "-----------" << std::endl;
        std::cout << "req dist  -->" << req.distance << " req th->" << req.theta << std::endl;
        std::cout << "initial:  x->" << init.x  << " y->" << init.y  << " th->" << init.th << std::endl;
        std::cout << "goal:     x->" << goal.x  << " y->" << goal.y  << " th->" << goal.th << std::endl;
        std::cout << "currrent: x->" << curr.x  << " y->" << curr.y  << " th->" << curr.th << std::endl;
        std::cout << "error:    x->" << error.x << " y->" << error.y << " th->" << error.th << std::endl;

        switch(state) {
            case SM_CORRECT_ANGLE:
                if(error.th > ANGLE_TOLERANCY) {
                    pubCmdVel.publish(getAngularMovement());
                } else {
                    state = SM_MOVE_ROBOT;
                }
            break;
            case SM_MOVE_ROBOT:
                if(error.x > DISTANCE_TOLERANCY) {
                    pubCmdVel.publish(getLinearMovement());
                } else {
                    state = SM_FINISH_MOVEMENT;
                }
            break;
            case SM_FINISH_MOVEMENT:
                pubCmdVel.publish(stop);
                std::cout << "Success distance reached! with dist.->" << req.distance << "  and angle.->" << req.theta << std::endl;
                res.done = true;
            break;
            default:
                std::cout << "An unexpected error has occurred :(" << std::endl;
        }

	    rate.sleep();
    }
    pubCmdVel.publish(stop);
    return res.done;
}

int main(int argc, char **argv) {
    std::cout << "Starting move_minibot_node_simul by Luis Nava..." << std::endl;
    ros::init(argc, argv, "move_minibot_node_simul");
    ros::NodeHandle nh;

    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::ServiceServer moveService = nh.advertiseService("move_robot", moveCallback);

    ros::spin();
    return 1;
}
