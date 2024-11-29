#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <mobile_base/MoveMinibot.h>
#include <mobile_base/profiles.h>
#include <mobile_base/OdomSetPoint.h>

struct robotPose {
  float x;
  float y;
  float th;
  float magnitude;
} curr, goal, error;

enum State {
    SM_MOVE_ROBOT,
    SM_CORRECT_ANGLE,
    SM_FINISH_MOVEMENT
};

ros::Publisher pubCmdVel;
ros::ServiceClient client;

float round2(double num) {
    return std::round(num * 1000) / 1000;
}

robotPose getTransform(std::string parentLink, std::string childLink) {
    tf2_ros::Buffer tfBuffer;
    robotPose transformPose;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform(parentLink, childLink, ros::Time(0), ros::Duration(1.0));
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::getEulerYPR(q, yaw, pitch, roll);
        transformPose.x  = transformStamped.transform.translation.x;
        transformPose.y  = transformStamped.transform.translation.y;
        transformPose.th = yaw;
        transformPose.magnitude = sqrt(pow(transformPose.x, 2) + pow(transformPose.y, 2));
        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << "move_minibot.-> parentLink: " << parentLink << "\tchildLink: " << childLink << std::endl;
        // std::cout << "move_minibot.-> x: " << transformPose.x << "\ty: " << transformPose.y << "\tth: " << transformPose.th << std::endl;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return transformPose;
}

bool setInitialPosition() {
    try{    
        robotPose currentPose = getTransform("map", "base_link");

        mobile_base::OdomSetPoint srv;
        srv.request.robot_x = currentPose.x;
        srv.request.robot_y = currentPose.y;
        srv.request.robot_w = currentPose.th;

        if (!client.call(srv)) {
            ROS_ERROR("move_minibot.->FAILED TO GET SERVICE /mobile_base/odom_set_point");
            return false;
        }
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
    return true;
}

robotPose getGoalPose(mobile_base::MoveMinibot::Request req) {
    robotPose goalPose;
    goalPose.x  = req.distance * cos(req.theta);
    goalPose.y  = req.distance * sin(req.theta);
    goalPose.th = req.theta;
    goalPose.magnitude = req.distance;

    if (goalPose.th >  M_PI)  goalPose.th -= 2 * M_PI;
    if (goalPose.th <= -M_PI) goalPose.th += 2 * M_PI;
    return goalPose;
}

robotPose getErrorPose() {
    robotPose errorPose;
    errorPose.x = fabs(goal.x) - fabs(curr.x);
    errorPose.y = fabs(goal.y) - fabs(curr.y);
    errorPose.th = goal.th - curr.th;
    errorPose.magnitude = sqrt(pow(errorPose.x, 2) + pow(errorPose.y, 2));
    if (goal.magnitude < 0) errorPose.magnitude = - errorPose.magnitude;

    return errorPose;
}

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {

    while(!setInitialPosition()) {}
    curr = getTransform("odom", "base_link");
    goal = getGoalPose(req);
    ros::Rate rate(10);

    State state = SM_CORRECT_ANGLE;

    int current_time = 0;
    res.done = req.distance == 0.0 && req.theta == 0.0;

    
    while(ros::ok() && !res.done) {
        // std::cout << "-------------------------------------------------------------------" << std::endl;
        // std::cout << "move_minibot.-> STATE MACHINE: " << state << std::endl;
        // std::cout << "move_minibot.-> ANGLE_TOLERANCY: " << ANGLE_TOLERANCY << "\tDISTANCE_TOLERANCY: " << DISTANCE_TOLERANCY << std::endl;
        // std::cout << std::endl;
        // std::cout << "move_minibot.-> Currrent-> x:" << round2(curr.x)  << "\ty:" << round2(curr.y)  << "\tth:" << round2(curr.th) << "\tmag:" << round2(curr.magnitude) << std::endl;
        // std::cout << "move_minibot.-> Goal->     x:" << round2(goal.x)  << "\ty:" << round2(goal.y)  << "\tth:" << round2(goal.th) << "\tmag:" << round2(goal.magnitude) << std::endl;
        // std::cout << "move_minibot.-> Error->    x:" << round2(error.x) << "\ty:" << round2(error.y) << "\tth:" << round2(error.th) << "\tmag:" << round2(error.magnitude) << std::endl;
        // std::cout << "move_minibot.-> Time execution->" << current_time << "ms" << std::endl;
        // std::cout << std::endl;

        if (!isRunning()) {
            res.done = true;
            std::cout << "************* MOVEMENT STOPPED **************" << std::endl;
            break;
        }

        curr = getTransform("odom", "base_link");
        error = getErrorPose();

        switch(state) {

            case SM_CORRECT_ANGLE:
                if(abs(error.th) >= ANGLE_TOLERANCY) {
                    pubCmdVel.publish(getAngularVelocity(error.th));
                } else {
                    state = SM_MOVE_ROBOT;
                }
            break;
            case SM_MOVE_ROBOT:
                if(abs(error.magnitude) >= DISTANCE_TOLERANCY) {
                    pubCmdVel.publish(getLinearVelocity(curr.magnitude, goal.magnitude, error.th));
                } else {
                    state = SM_FINISH_MOVEMENT;
                }
            break;
            case SM_FINISH_MOVEMENT:
                pubCmdVel.publish(stop());
                std::cout << "Success distance reached! with dist.->" << req.distance << "  and angle.->" << req.theta << std::endl;
                res.done = true;
            break;
            default:
                std::cout << "An unexpected error has occurred :(" << std::endl;
        }
        current_time += 10;
        if (current_time > MAX_TIME_LIMIT) {
            std::cout << "MOVEMENT HAS EXCEEDED TIME MAX" << std::endl;
            res.done = true;
        }

	    rate.sleep();
    }
    pubCmdVel.publish(stop());
    return res.done;
}

int main(int argc, char **argv) {
    std::cout << "Starting move_minibot_node by Luis Nava..." << std::endl;
    ros::init(argc, argv, "move_minibot_node");
    ros::NodeHandle nh;
    if(!setParameters()) return -1;

    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/mobile_base/cmd_vel", 1);
    client = nh.serviceClient<mobile_base::OdomSetPoint>("/mobile_base/odom_set_point");
    ros::ServiceServer moveService = nh.advertiseService("/mobile_base/move_to_pose", moveCallback);

    ros::spin();
    return 0;
}
