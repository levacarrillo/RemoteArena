#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

double sharp_distance[3] = {0.7, 0.7, 0.7};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_node");
    ros::NodeHandle nh;
    ros::Rate loop(30);

    ros::Publisher  scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);


	unsigned int num_readings = 3;
	double laser_frequency = 40;
	double ranges[num_readings];
	double intensities[num_readings];

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

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
			intensities[i] = 100;
		}


		ros::Time scan_time = ros::Time::now();

        sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_link";
		//scan.angle_min = - 2.35619;
		scan.angle_min = - 0.7853;
		//scan.angle_max =    3.1416;
		scan.angle_max = 3.1416;

		scan.angle_increment = 0.785398;
		scan.time_increment = (1 / laser_frequency) / (num_readings);
		scan.range_min = 0.0;
		scan.range_max = 100.0;

		scan.ranges.resize(num_readings);
		scan.intensities.resize(num_readings);

		for(unsigned int i = 0; i < num_readings; ++i) {

			scan.ranges[i] = ranges[i];
			scan.intensities[i] = intensities[i];
		}

        scan_pub.publish(scan);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}