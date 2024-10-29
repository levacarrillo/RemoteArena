#include <ros/ros.h>
#include <motion_planner/LightReadings.h>
#include <motion_planner/motion_planner_utilities.h>
#include <../utilities/structures.h>
#include <../state_machines/light_follower.h>


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "motion_planner_node");
    ros::NodeHandle nh;

    ros::ServiceClient light_client = nh.serviceClient<motion_planner::LightReadings>("light_readings");
    motion_planner::LightReadings srv;
    ros::Rate rate(20);

    bool stop_running = false;
    float light_max_intensity;
    float light_readings[8];

    movement movement;
    float max_advance    =  0.3;
    float max_turn_angle =  0.3;

    enum Behavior {
        SM_LIGHT_FOLLOWER,
        SM_DESTINATION,
        SM_AVOID_OBSTACLES,
        SM_AVOIDANCE_DESTINATION,
        SM_ORACLE_CLIPS,
        DFS,
        DIJKSTRA,
        USER_SM,
        ACTION_PLANNER,
        LINE_FOLLOWER
    };

    Behavior behavior = SM_LIGHT_FOLLOWER;

    MotionPlannerUtilities motionPlannerUtilities(nh);

    while(ros::ok()) {
        if (!stop_running) {
            if (light_client.call(srv)) {
                light_max_intensity = srv.response.max_intensity;
                for (size_t i=0; i<srv.response.light_readings.size(); i++) {
                    light_readings[i] = srv.response.light_readings[i];
                }
            } else {
                ROS_ERROR("FAILED TO CALL SERVICE /light_readings");
            }

            switch(behavior) {
                case SM_LIGHT_FOLLOWER:
                    stop_running = light_follower(light_max_intensity, light_readings, &movement, max_advance);
                break;
                default:
                    std::cout << " *************** NO BEHAVIOR DEFINED *************** " << std::endl;
                    movement.twist = 0.0;
                    movement.advance = 0.0;
                break;
            }

            std::cout << "\n \n  MOTION PLANNER \n____________________________\n" << std::endl;
            // std::cout << "Light" << std::endl;
            // std::cout << "Robot: " << std::endl;
            // std::cout << "Step" << std::endl;
            std::cout << "Movement: twist: " << movement.twist << " advance: " << movement.advance << "\n" << std::endl;

        
            motionPlannerUtilities.move_robot(movement.twist, movement.advance);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}