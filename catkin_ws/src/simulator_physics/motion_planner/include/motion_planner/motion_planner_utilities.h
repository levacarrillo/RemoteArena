#ifndef MOTION_PLANNER_UTILITIES_H
#define MOTION_PLANNER_UTILITIES_H

#include <ros/ros.h>
#include <mobile_base/MoveMinibot.h>

class MotionPlannerUtilities {
    private:
        ros::NodeHandle &nh_;
        ros::ServiceClient client;
    public:
        MotionPlannerUtilities(ros::NodeHandle &nh) : nh_(nh) {
            std::cout << "standing..." << std::endl;
            client = nh_.serviceClient<mobile_base::MoveMinibot>("move_robot");
        }

        void move_robot(float theta, float advance) {
            mobile_base::MoveMinibot srv;
            srv.request.theta    = theta;
            srv.request.distance = advance;
            if (client.call(srv))
            {
                if(srv.response.done)
                    printf("Robot move done \n");
                else
                    printf("Robot move fail \n");
            }
            else
            {
                ROS_ERROR("Failed to call service move_robot");

            }
        }

};

// void move_robot(float theta, float advance) {
//     std::cout << "Theta:->" << theta << std::endl;
//     std::cout << "Advance:->" << advance << std::endl;

//     ros::NodeHandle n;
//     ros::ServiceClient client;
//     mobile_base::MoveMinibot srv;
//     client = n.serviceClient<mobile_base::MoveMinibot>("move_robot");
//     srv.request.theta    = theta;
//     srv.request.distance = advance;

//     if (client.call(srv))
//     {
//         if(srv.response.done)
//             printf("Robot move done \n");
//         else
//             printf("Robot move fail \n");
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service move_robot");

//     }
// }

#endif
