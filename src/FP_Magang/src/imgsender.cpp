// Node A - capture_and_send_image.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_sender_node");
    ros::NodeHandle nh;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 10);
    cv::VideoCapture cap("/home/gilbran/MagangIRIS/MagangIRIS-FP/img/bola1.jpg");

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open image");
        return -1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            ROS_WARN("Empty frame captured");
            continue;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        image_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    cap.release();
    return 0;
}
