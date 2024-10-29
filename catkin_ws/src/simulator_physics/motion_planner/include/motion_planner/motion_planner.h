#ifndef MOTION_PLANNER_UTILITIES_H
#define MOTION_PLANNER_UTILITIES_H

#include <ros/ros.h>
#include <mobile_base/MoveMinibot.h>

class MotionPlanner {
    private:
        ros::NodeHandle &nh_;
        ros::ServiceClient client;
        float max_advance;
        float max_turn_angle;
    public:
        MotionPlanner(ros::NodeHandle &nh) : nh_(nh) {
            client = nh_.serviceClient<mobile_base::MoveMinibot>("move_robot");
        }

        float get_max_advance() {
            if (!nh_.getParam("/max_advance", max_advance)) {
                ROS_ERROR("FAILED TO GET PARAMETER /max_advance OF ROBOT");
            }
            return max_advance;
        }
        
        float get_max_turn_angle() {
            if (!nh_.getParam("/max_turn_angle", max_turn_angle)) {
                ROS_ERROR("FAILED TO GET PARAMETER /max_turn_angle OF ROBOT");
            }
            return max_turn_angle;
        }

        void move_robot(float theta, float advance) {
            mobile_base::MoveMinibot srv;
            srv.request.theta    = theta;
            srv.request.distance = advance;
            if (client.call(srv)) {
                if(srv.response.done) printf("Robot movement done \n");
                else printf("Robot movement fail \n");
            } else {
                ROS_ERROR("Failed to call service /move_robot");
            }
        }
};

#endif
