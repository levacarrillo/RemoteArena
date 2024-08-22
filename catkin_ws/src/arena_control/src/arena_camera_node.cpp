#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;

int camera_id = 0;


int main(int argc, char** argv)
{
    cout << "Starting arena_camera_node by Luis Nava..." << endl;

    ros::init(argc, argv, "arena_camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);

    if(ros::param::has("/camera_id")) ros::param::get("/camera_id", camera_id);

    cv::VideoCapture cap(camera_id);

    if(!cap.isOpened()) return 1;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);
    
    while (nh.ok()) {
    cap >> frame;
    
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}
