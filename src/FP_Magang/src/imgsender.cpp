// Node A - capture_and_send_image.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_sender_node");
    ros::NodeHandle nh;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 10);
    Mat photo = imread("/home/gilbran/MagangIRIS/MagangIRIS-FP/src/FP_Magang/src/img/bola2.jpg");
    Mat resized;
    resize(photo, resized, Size(900, 600));


    ros::Rate loop_rate(10);
    while (ros::ok()) {

        if (resized.empty()) {
            continue;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized).toImageMsg();

        image_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
