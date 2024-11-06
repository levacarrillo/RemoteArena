#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <hardware/LightReadings.h>

#define ROBOT_RADIUS 0.08

float sensor_angle[8];

struct {
    float x;
    float y;
} robot_distance_to_light, robot_position, light_bulb_position;


bool light_callback(hardware::LightReadings::Request &req, hardware::LightReadings::Response &res) {
    float light_readings[8];
    float max_intensity = 0;
    float sensor_max_intensity = 0;

    for (size_t i=0; i<8; i++) {
        float light_distance_x = robot_distance_to_light.x - ROBOT_RADIUS * std::cos(sensor_angle[i]);
        float light_distance_y = robot_distance_to_light.y - ROBOT_RADIUS * std::sin(sensor_angle[i]);
        res.light_readings[i] = light_readings[i] = 1 / sqrt(pow(light_distance_x, 2) + pow(light_distance_y, 2));

        if (light_readings[i] > max_intensity) {
            max_intensity = light_readings[i];
            sensor_max_intensity = i;
        }
    }

    res.sensor_max_intensity = sensor_max_intensity;
    res.max_intensity = max_intensity;

    return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "light_sensors_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/light_readings", light_callback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);

    static std::string light[2] = {"light_bulb_1", "light_bulb_2"};
    ros::Duration(2);

    while(ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;

        std::vector<bool> light_status;
        nh.getParam("/light_bulbs", light_status);

        try {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            tf2::Quaternion q;
            tf2::fromMsg(transformStamped.transform.rotation, q);
            double roll, pitch, yaw;
            tf2::getEulerYPR(q, yaw, pitch, roll);

            robot_position.x =  transformStamped.transform.translation.x;
            robot_position.y =  transformStamped.transform.translation.y;
            
            int selected = -1;
            for(int i=0; i<light_status.size(); i++) {
                if(light_status[i] == 1) {
                    selected = i;
                }
            }
            if(selected >= 0)
                transformStamped = tfBuffer.lookupTransform("map", light[selected], ros::Time(0));

            light_bulb_position.x =  transformStamped.transform.translation.x;
            light_bulb_position.y =  transformStamped.transform.translation.y;

            robot_distance_to_light.x = light_bulb_position.x - robot_position.x;
            robot_distance_to_light.y = light_bulb_position.y - robot_position.y;

            // std::cout << "Distance to light x->" << robot_distance_to_light.x;
            // std::cout << "\ty->" << robot_distance_to_light.y << std::endl;

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