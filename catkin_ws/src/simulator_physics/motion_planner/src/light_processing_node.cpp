#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <motion_planner/LightReadings.h>

#define ROBOT_RADIUS 0.1
float THRESHOLD = 200.0;

float sensor_angle[8];

struct {
    float x;
    float y;
} robot_distance_to_light;


bool light_callback(motion_planner::LightReadings::Request &req, motion_planner::LightReadings::Response &res) {
    float light_readings[8];
    int id_sensor_max;
    float max_intensity = 0;
    bool threshold_reached = false;

    for (int i=0; i<8; i++) {
        float light_distance_x = robot_distance_to_light.x - ROBOT_RADIUS * std::cos(sensor_angle[i]);
        float light_distance_y = robot_distance_to_light.y - ROBOT_RADIUS * std::sin(sensor_angle[i]);
        res.light_readings[i] = light_readings[i] = 1 / sqrt(pow(light_distance_x, 2) + pow(light_distance_y, 2));

        if (light_readings[i] > max_intensity) {
            id_sensor_max = i;
            max_intensity = light_readings[i];
        }
    }
    if (max_intensity >= THRESHOLD) {
        threshold_reached = true;
    }

    // res.id_sensor_max = id_sensor_max;
    res.max_intensity = max_intensity;
    // res.threshold_reached = threshold_reached;
    return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "light_processing_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/light_readings", light_callback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);

    while(ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;

        try {
            transformStamped = tfBuffer.lookupTransform("base_link", "light_bulb", ros::Time(0));
            tf2::Quaternion q;
            tf2::fromMsg(transformStamped.transform.rotation, q);
            double roll, pitch, yaw;
            tf2::getEulerYPR(q, yaw, pitch, roll);

            robot_distance_to_light.x =  transformStamped.transform.translation.x;
            robot_distance_to_light.y =  transformStamped.transform.translation.y;

            for (int i=0; i<8; i++) {
                sensor_angle[i] = i * M_PI / 4 + yaw;
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}