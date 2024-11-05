#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "FP_Magang/PC2BS.h"
#include "FP_Magang/coordinate.h"
#include "FP_Magang/BS2PC.h"

#define PI 3.141592654
#define RAD 0.0174532925199 // PI/180
#define DEG 57.295779513082 // 180/PI

ros::Publisher pub;
ros::Subscriber stat_sub;
ros::Subscriber th_sub;
ros::Subscriber coord_sub;
ros::Timer init;
ros::Timer loop;
FP_Magang::PC2BS cmd;

int basestatus;
int resstats, prevstats, stats, xtu, ytu;
double motor1 = 0, motor2 = 0, motor3 = 0;
double angle1 = 0, angle2 = 120, angle3 = 240;
double angle1Rad = angle1 * RAD;
double angle2Rad = angle2 * RAD;
double angle3Rad = angle3 * RAD;
double bx = 0, by = 0, vx = 0, vy = 0, vth = 0, th;
char keyb;

void inkey() {
    static struct termios oldt;
    static struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);           // Simpan pengaturan lama
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // Non-blocking input
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // Terapkan pengaturan baru
    keyb = getchar();  // Baca karakter (non-blocking)
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // Kembalikan pengaturan lama
}

void inversekinematic(double vx, double vy, double vth) {
    motor1 = vx * cos(angle1Rad) + vy * sin(angle1Rad) + vth;
    motor2 = vx * cos(angle2Rad) + vy * sin(angle2Rad) + vth;
    motor3 = vx * cos(angle3Rad) + vy * sin(angle3Rad) + vth;
    ROS_INFO("motor1=%.2f motor2=%.2f motor3=%.2f", motor1, motor2, motor3);

}

void robotspeed(double motor_input1, double motor_input2, double motor_input3) {
    vx = (2.0 / 3.0) * (motor_input1 * cos(angle1Rad) + motor_input2 * cos(angle2Rad) + motor_input3 * cos(angle3Rad));
    vy = (2.0 / 3.0) * (motor_input1 * sin(angle1Rad) + motor_input2 * sin(angle2Rad) + motor_input3 * sin(angle3Rad));
    vth = (motor_input1 + motor_input2 + motor_input3) / 3.0;
    double tempvx = vx * cos(th) - vy * sin(th);
    double tempvy = vx * cos(th) + vy * sin(th);
    vx = tempvx;
    vy = tempvy;

    inversekinematic(vx, vy, vth);
    ROS_INFO("vx=%.2f vy=%.2f vth=%.2f", vx, vy, vth);
}

void status1() {
    int speed = 250;
    inkey();

    switch (keyb) {
    case 'w': // Move forward
        robotspeed(0, speed, -speed);
        break;
    case 's': // Move backward
        robotspeed(0, -speed, speed);
        break;
    case 'd': // Move right
        robotspeed(speed, -(speed * 0.5), -(speed * 0.5));
        break;
    case 'a': // Move left
        robotspeed(-speed, (-speed * 0.5), (speed * 0.5));
        break;
    case 'q': // Rotate counterclockwise
        robotspeed(speed, speed, speed);
        break;
    case 'e': // Rotate clockwise
        robotspeed(-speed, -speed, -speed);
        break;
    default:
        robotspeed(0, 0, 0);
        return;
    }
}

void statusCallback(const FP_Magang::BS2PC::ConstPtr& msg) {
    ROS_INFO("Received status = %d", static_cast<int>(msg->status));
    if (prevstats != msg->status) {
        switch (static_cast<int>(msg->status)) {
        case 1:
            ROS_INFO("Status 1");
            basestatus = 1111;
            break;
        case 2:
            ROS_INFO("Status 2");
            basestatus = 2222;
            break;
        case 3:
            ROS_INFO("Status 3");
            basestatus = 3333;
            break;
        case 4:
            ROS_INFO("Status 4");
            basestatus = 4444;
            break;
        default:
            basestatus = 0;
            break;
        }
    }
    stats = msg->status;
    prevstats = msg->status;
    xtu = msg->tujuan_x;
    ytu = msg->tujuan_y;
}

void thetaCallback(const FP_Magang::BS2PC::ConstPtr& msg) {
    th = msg->th;
    while (th > 180 || th < -180) {
        if (th > 180) {
            th = -th + 180;
        }
        else if (th < -180) {
            th = -th - 180;
        }
    }
    th = th * RAD;
    ROS_INFO("Received theta: %.0f", th * DEG);

}

void coordinateCallback(const FP_Magang::coordinate::ConstPtr& msg) {
    if (basestatus != 1) {
        bx = msg->x;
        by = msg->y;
    }
    ROS_INFO("Received coordinates: x=%.2f, y=%.2f", msg->x, msg->y);
}

void loopCallback(const ros::TimerEvent&) {
    switch (basestatus) {
    case 1111:
        status1();
        break;
        // case 2222:
        //     status2();
        //     break;
        // case 3333:
        //     status3();
        //     break;
        // case 4444:
        //     status4();
        //     break;
    default:
        break;
    }

    FP_Magang::PC2BS pesan;
    pesan.motor1 = motor1;
    pesan.motor2 = motor2;
    pesan.motor3 = motor3;
    pesan.bola_x = bx;
    pesan.bola_y = by;
    pub.publish(pesan);
}

void resetCallback(const ros::TimerEvent&) {
    if (resstats != prevstats) {
        FP_Magang::PC2BS reset;
        motor1 = 0;
        motor2 = 0;
        motor3 = 0;
        reset.motor1 = motor1;
        reset.motor2 = motor2;
        reset.motor3 = motor3;
        pub.publish(reset);
        resstats = prevstats;
    }
}

// Main node setup and event loop
int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;

    init = nh.createTimer(ros::Duration(0.02), resetCallback);
    stat_sub = nh.subscribe("/bs2pc", 50, statusCallback);
    th_sub = nh.subscribe("/bs2pc", 50, thetaCallback);
    coord_sub = nh.subscribe("/ball_coordinate", 50, coordinateCallback);
    loop = nh.createTimer(ros::Duration(0.02), loopCallback);
    pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 50);

    ros::spin();
    return 0;
}