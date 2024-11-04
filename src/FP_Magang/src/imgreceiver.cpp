// Node B - receive_and_detect_orange.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <FP_Magang/coordinate.h>  // Include the custom message header

using namespace cv;

ros::Publisher coord_pub;

// Callback function to process the received image and detect orange objects
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert ROS Image message to OpenCV image
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Convert the image from BGR to HSV color space
        Mat hsv_img;
        cvtColor(img, hsv_img, COLOR_BGR2HSV);

        // Threshold the image to get only orange colors
        Mat mask;
        inRange(hsv_img, Scalar(5, 150, 150), Scalar(15, 255, 255), mask);

        // Find contours of orange objects
        std::vector<std::vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Loop through each contour to find the centroid and publish coordinates
        for (size_t i = 0; i < contours.size(); ++i) {
            Moments m = moments(contours[i]);
            if (m.m00 > 0) {
                // Calculate the centroid coordinates
                float x = m.m10 / m.m00;
                float y = m.m01 / m.m00;

                // Create and publish the coordinate message
                FP_Magang::coordinate coord_msg;
                coord_msg.x = x;
                coord_msg.y = y;
                coord_pub.publish(coord_msg);

                ROS_INFO("Orange object detected at coordinates: x = %f, y = %f", x, y);
            }
        }

        // (Optional) Display the result (for debugging)
        imshow("Original Image", img);
        waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "orange_detection_node");
    ros::NodeHandle nh;

    // Publisher for coordinates of orange objects
    coord_pub = nh.advertise<FP_Magang::coordinate>("/ball_coordinate", 10);

    // Subscriber for receiving images
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 10, imageCallback);

    ros::spin();  // Keep the node alive to listen to incoming messages
    return 0;
}
