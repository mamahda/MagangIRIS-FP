#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <FP_Magang/coordinate.h> 

using namespace cv;

ros::Publisher coord_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat hsv_img;
        cvtColor(img, hsv_img, COLOR_BGR2HSV);

        Mat mask;
        inRange(hsv_img, Scalar(5, 150, 150), Scalar(15, 255, 255), mask);

        std::vector<std::vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); ++i) {
            Moments m = moments(contours[i]);
            if (m.m00 > 0) {
                float x = m.m10 / m.m00;
                float y = m.m01 / m.m00;

                FP_Magang::coordinate coord_msg;
                coord_msg.x = x;
                coord_msg.y = y;
                coord_pub.publish(coord_msg);

                // ROS_INFO("ball coordinates: x = %f, y = %f", x, y);
            }
        }
        waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "orange_detection_node");
    ros::NodeHandle nh;

    coord_pub = nh.advertise<FP_Magang::coordinate>("/ball_coordinate", 10);

    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 10, imageCallback);

    ros::spin();
    return 0;
}
