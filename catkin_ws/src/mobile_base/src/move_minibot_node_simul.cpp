#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/MoveMinibot.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mobile_base/profiles.h>;

struct robotPose {
  float x;
  float y;
  float th;
  float magnitude;
} init, goal, curr, error;

enum State {
    SM_MOVE_ROBOT,
    SM_CORRECT_ANGLE,
    SM_FINISH_MOVEMENT
};

float ANGLE_TOLERANCY = 0.1876;
float DISTANCE_TOLERANCY = 0.08;
ros::Publisher pubCmdVel;

robotPose getCurrentPose(tf2_ros::Buffer& tfBuffer) {
    robotPose currentPose;
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time::now(), ros::Duration(0.1));
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::getEulerYPR(q, yaw, pitch, roll);

        currentPose.x = transformStamped.transform.translation.x;
        currentPose.y = transformStamped.transform.translation.y;
        currentPose.th = yaw;
        currentPose.magnitude = sqrt(pow(currentPose.x, 2) + pow(currentPose.y, 2));

        if (yaw > M_PI)   currentPose.th -= 2 * M_PI;
        if (yaw <= -M_PI) currentPose.th += 2 * M_PI;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
        std::cout << "currrent: x->" << currentPose.x  << "\ty->" << currentPose.y  << "\tth->" << currentPose.th << "\tmag->" << std::round(currentPose.magnitude) << std::endl;
    return currentPose;
}

robotPose getGoalPose(mobile_base::MoveMinibot::Request req) {
    robotPose goalPose;
    goalPose.x = init.x + req.distance * cos(init.th + req.theta);
    goalPose.y = init.y + req.distance * sin(init.th + req.theta);
    goalPose.th = init.th + req.theta;
    goalPose.magnitude = sqrt(pow(goalPose.x, 2) + pow(goalPose.y, 2));
    if (req.distance < 0) goalPose.magnitude = - goalPose.magnitude;
    if (goalPose.th >  M_PI)  goalPose.th -= 2 * M_PI;
    if (goalPose.th <= -M_PI) goalPose.th += 2 * M_PI;
    return goalPose;
}

robotPose getErrorPose() {
    robotPose errorPose;
    errorPose.x = goal.x - curr.x;
    errorPose.y = goal.y - curr.y;
    errorPose.th = goal.th - curr.th;
    errorPose.magnitude = sqrt(pow(errorPose.x, 2) + pow(errorPose.y, 2));
    if (goal.magnitude < 0) errorPose.magnitude = - errorPose.magnitude;
    return errorPose;
}

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    
    curr = getCurrentPose(tfBuffer);
    init = getCurrentPose(tfBuffer);
    goal = getGoalPose(req);

    State state = SM_CORRECT_ANGLE;
    Profile angularProfile = RECTANGULAR;
    Profile linearProfile = RECTANGULAR;

    std::cout << "*********************************************************************" << std::endl;
    std::cout << "req dist  -->" << req.distance << "\treq th->" << req.theta << std::endl;
    std::cout << "initial:  x->" << init.x << "\ty->" << init.y  << "\tth->" << init.th << "\tmag->" << init.magnitude << std::endl;
    std::cout << "currrent: x->" << curr.x  << "\ty->" << curr.y  << "\tth->" << curr.th << "\tmag->" << curr.magnitude << std::endl;
    std::cout << "goal:     x->" << goal.x  << "\ty->" << goal.y  << "\tth->" << goal.th << "\tmag->" << goal.magnitude << std::endl;
    std::cout << "error:    x->" << error.x << "\ty->" << error.y << "\tth->" << error.th << "\tmag->" << error.magnitude << std::endl;
    std::cout << "*********************************************************************" << std::endl;
    std::cout << std::endl;

    while(ros::ok() && !res.done) {
        curr = getCurrentPose(tfBuffer);
        error = getErrorPose();
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << "req dist  -->" << req.distance << "\treq th->" << req.theta << std::endl;
        std::cout << "initial:  x->" << init.x  << "\ty->" << init.y  << "\tth->" << init.th << "\tmag->" << init.magnitude << std::endl;
        std::cout << "currrent: x->" << curr.x  << "\ty->" << curr.y  << "\tth->" << curr.th << "\tmag->" << curr.magnitude << std::endl;
        std::cout << "goal:     x->" << goal.x  << "\ty->" << goal.y  << "\tth->" << goal.th << "\tmag->" << goal.magnitude << std::endl;
        std::cout << "error:    x->" << error.x << "\ty->" << error.y << "\tth->" << error.th << "\tmag->" << error.magnitude << std::endl;
        std::cout << std::endl;

        switch(state) {
            case SM_CORRECT_ANGLE:
                if(abs(error.th) > ANGLE_TOLERANCY) {
                    pubCmdVel.publish(getAngularVelocity(init.th, curr.th, goal.th, error.th, angularProfile));
                } else {
                    state = SM_MOVE_ROBOT;
                }
            break;
            case SM_MOVE_ROBOT:
                if(abs(error.magnitude) > DISTANCE_TOLERANCY) {
                    pubCmdVel.publish(getLinearVelocity(error.magnitude, linearProfile));
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

	    rate.sleep();
    }
    pubCmdVel.publish(stop());
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
