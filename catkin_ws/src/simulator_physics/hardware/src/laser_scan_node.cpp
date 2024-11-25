#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <random_numbers/random_numbers.h>

double sharp_distance[3] = {0.7, 0.7, 0.7};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_node");
    ros::NodeHandle nh;
    ros::Rate loop(30);

    ros::Publisher  scan_pub = nh.advertise<sensor_msgs::LaserScan>("/hardware/scan", 50);


	unsigned int num_readings = 3;
	double laser_frequency = 40;
	double ranges[num_readings];
	float noise = 0;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

	if(nh.hasParam("/hardware/lasers_noise")) 
		nh.getParam("/hardware/lasers_noise", noise);

	random_numbers::RandomNumberGenerator rnd;

    while(ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;

        try{
            transformStamped = tfBuffer.lookupTransform("base_link", "laser_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

		for(int i=0; i<3; i++) {
			ranges[i] = sharp_distance[i];
		}


		ros::Time scan_time = ros::Time::now();

        sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_link";
		scan.angle_min = - M_PI / 4;
		scan.angle_max = M_PI;

		scan.angle_increment = M_PI / 4;
		scan.time_increment = 1 / ( laser_frequency * num_readings);
		scan.range_min = 0.0;
		scan.range_max = 100.0;

		scan.ranges.resize(num_readings);

		for(unsigned int i = 0; i < num_readings; ++i) {
			scan.ranges[i] = ranges[i] + rnd.uniformReal(-noise, noise);;
		}

        scan_pub.publish(scan);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}