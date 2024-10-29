#include <ros/ros.h>
#include <../utilities/structures.h>
#include <motion_planner/LightReadings.h>
#include <motion_planner/motion_planner.h>
#include <motion_planner/motion_planner_utilities.h>
#include <../state_machines/light_follower.h>
#include <../state_machines/sm_destination.h>


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "motion_planner_node");
    ros::NodeHandle nh;

    ros::ServiceClient light_client = nh.serviceClient<motion_planner::LightReadings>("light_readings");
    motion_planner::LightReadings srv;
    ros::Rate rate(20);

    bool run_algorithm = false;

    float max_intensity;
    float light_readings[8];
    int q_light;
    int next_state = 1;

    movement movement;

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

    MotionPlanner mp(nh);

    while(ros::ok()) {
        if (!nh.getParam("/run_algorithm", run_algorithm)) {
            ROS_ERROR("FAILED TO GET PARAMETER /run_algorithm");
        }

        if (run_algorithm) {

            if (light_client.call(srv)) {
                max_intensity = srv.response.max_intensity;
                for (size_t i=0; i<srv.response.light_readings.size(); i++) {
                    light_readings[i] = srv.response.light_readings[i];
                }
                q_light = quantize_light(light_readings);
            } else {
                ROS_ERROR("FAILED TO CALL SERVICE /light_readings");
            }
            for (int i=0; i<8;i++){
                std::cout << light_readings[i] << std::endl;
            }
            switch(behavior) {
                case SM_LIGHT_FOLLOWER:
                    run_algorithm = !light_follower(max_intensity, light_readings, &movement, mp.get_max_advance());
                break;
                case SM_DESTINATION:
                    run_algorithm = !sm_destination(max_intensity, q_light, &movement, &next_state, mp.get_max_advance(), mp.get_max_turn_angle());
                break;
                default:
                    std::cout << " *************** NO BEHAVIOR DEFINED *************** " << std::endl;
                    movement.twist = 0.0;
                    movement.advance = 0.0;
                break;
            }

            if (!run_algorithm) { 
                nh.setParam("/run_algorithm", false);
                ros::Duration(10);
            }

            std::cout << "\n \n  MOTION PLANNER \n____________________________\n" << std::endl;
            // std::cout << "Light" << std::endl;
            // std::cout << "Robot: " << std::endl;
            // std::cout << "Step" << std::endl;
            std::cout << "Movement: twist: " << movement.twist << " advance: " << movement.advance << "\n" << std::endl;

        
            mp.move_robot(movement.twist, movement.advance);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}