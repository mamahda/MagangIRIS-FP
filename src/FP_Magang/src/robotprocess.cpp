#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "FP_Magang/PC2BS.h"
#include "FP_Magang/coordinate.h"
#include "FP_Magang/BS2PC.h"

ros::Publisher pub;
ros::Subscriber stat_sub;
ros::Subscriber coord_sub;
FP_Magang::PC2BS cmd;

void statusCallback(const FP_Magang::BS2PC msg) {
    ROS_INFO("Received status: %.0f", msg.status);
    ROS_INFO("Target coordinates: x=%.2f, y=%.2f", msg.tujuan_x, msg.tujuan_y);
}

void coordinateCallback(const FP_Magang::coordinate msg) {
    ROS_INFO("Received coordinates: x=%.2f, y=%.2f, z=%.2f", msg.x, msg.y, msg.z);
}

void robot(double motor1, double motor2, double motor3) {
    cmd.motor1 = motor1;
    cmd.motor2 = motor2;
    cmd.motor3 = motor3;
    pub.publish(cmd);
    ros::Duration(1).sleep();
    cmd.motor1 = 0;
    cmd.motor2 = 0;
    cmd.motor3 = 0;
    pub.publish(cmd);
}

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int c = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

// Main node setup and event loop
int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;

    pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 50);
    stat_sub = nh.subscribe("/bs2pc", 50, statusCallback);
    coord_sub = nh.subscribe("/ball_coordinate", 50, coordinateCallback);

    ROS_INFO("Node started and waiting for messages...");

    while (ros::ok()) {
        robot(0, 500, -500);
        char key = getch();
        switch (key) {
        case 'w': robot(0, 5, -5); break;
        case 's': robot(0, -5, 5); break;
        case 'a': robot(-5, 2.5, 2.5); break;
        case 'd': robot(5, -2.5, -2.5); break;
        default: break;
        }

        // Buat pesan dan publikasikan
        your_package::coordinate msg;
        msg.x = current_value;
        msg.y = current_value;
        pub.publish(msg);

        ROS_INFO("Publishing: x=%.2f, y=%.2f", msg.x, msg.y);


        ros::spinOnce();
    }
    return 0;
}