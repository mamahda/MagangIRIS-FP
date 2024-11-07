#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
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
ros::Subscriber robot_sub;
ros::Subscriber enc_sub;
ros::Timer init;
ros::Timer loop;
FP_Magang::PC2BS cmd;

int basestatus;
int resstats, prevstats, stats;
int speed = 250;
double posth = 0, posx = 0, posy = 0;
double motor1 = 0, motor2 = 0, motor3 = 0;
double angle1 = 0, angle2 = 120, angle3 = 240, anglel = 45, angler = 315;
double angle1Rad, angle2Rad, angle3Rad, anglelRad, anglerRad;
double bx = 0, by = 0, vx = 0, vy = 0, vth = 0, th;
double rx, ry, tx, ty;
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

void motorspeed(double vy, double vx, double vth = 0) {
    motor1 = vy * -1 + vx * 0 + vth;
    motor2 = vy * 0.5 + vx * 0.86602540378 + vth;
    motor3 = vy * 0.5 - vx * 0.86602540378 + vth;
    // ROS_INFO("motor1=%.2f motor2=%.2f motor3=%.2f", motor1, motor2, motor3);

}

void robotspeed(double motor_input1, double motor_input2, double motor_input3) {
    vy = (2.0 / 3.0) * (motor_input1 * cos(angle1Rad) + motor_input2 * cos(angle2Rad) + motor_input3 * cos(angle3Rad));
    vx = (2.0 / 3.0) * (motor_input1 * sin(angle1Rad) + motor_input2 * sin(angle2Rad) + motor_input3 * sin(angle3Rad));
    vth = (motor_input1 + motor_input2 + motor_input3) / 3.0;
    // ROS_INFO("vy=%.2f vx=%.2f vth=%.2f", vy, vx, vth);
    motorspeed(vy, vx, vth);
}

void status1() {
    inkey();

    switch (keyb) {
    case 'w':
        robotspeed(0, speed, -speed);
        break;
    case 's':
        robotspeed(0, -speed, speed);
        break;
    case 'd':
        robotspeed(speed, -(speed * 0.5), -(speed * 0.5));
        break;
    case 'a':
        robotspeed(-speed, (speed * 0.5), (speed * 0.5));
        break;
    case 'q':
        robotspeed(speed, speed, speed);
        break;
    case 'e':
        robotspeed(-speed, -speed, -speed);
        break;
    default:
        robotspeed(0, 0, 0);
        return;
    }
}

void status2() {
    static float v_x2 = speed * 0.5;
    static float v_y2 = speed * 0.5;

    double sudut = atan2(by - posy, bx - posx);
    posth = sudut * DEG;

    double distance = sqrt(pow(bx - posx, 2) + pow(by - posy, 2));
    ROS_INFO("Current Position: (%.2f, %.2f), Target: (%.2f, %.2f), Distance: %.2f", posx, posy, bx, by, distance);

    if (distance > 0) {
        vx = v_x2 * cos(sudut);
        vy = v_y2 * sin(sudut);

        posx += (vx / 50);
        posy += (vy / 50);

        motorspeed(-vx, vy);
    }
}

void status3() {
    static float v_x2 = speed * 0.5;
    static float v_y2 = speed * 0.5;

    // Calculate angle and heading toward the target (tx, ty)
    double sudut = atan2(ty - posy, tx - posx);
    posth = sudut * DEG;

    double distance = sqrt(pow(tx - posx, 2) + pow(ty - posy, 2));
    ROS_INFO("Current Position: (%.2f, %.2f), Target: (%.2f, %.2f), Distance: %.2f", posx, posy, tx, ty, distance);

    if (distance > 5) { // Adjust threshold distance as needed
        vx = v_x2 * cos(sudut);
        vy = v_y2 * sin(sudut);

        posx += (vx / 50);
        posy += (vy / 50);

        motorspeed(-vx, vy);
    }
    else {
        vx = 0;
        vy = 0;
        motorspeed(0, 0); // Stop robot when close enough
    }
}

void statusCallback(const FP_Magang::BS2PC::ConstPtr& msg) {
    // ROS_INFO("Received status = %d", static_cast<int>(msg->status));
    stats = static_cast<int>(msg->status);
    if (prevstats != stats) {
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
}

void thetaCallback(const FP_Magang::BS2PC::ConstPtr& msg) {
    th = msg->th;
    while (th > 180 || th < -180) {
        if (th > 180) {
            th = -th + 180;
        }
        else if (th < -180) {
            th = th + 360;
        }
    }
    // ROS_INFO("Received theta: %.0f", th);
    angle1 = 0 + th;
    angle2 = 120 + th;
    angle3 = 240 + th;
    angle1Rad = angle1 * RAD;
    angle2Rad = angle2 * RAD;
    angle3Rad = angle3 * RAD;
    anglelRad = anglel * RAD;
    anglerRad = angler * RAD;
    // ROS_INFO("angle1=%.2f angle2=%.2f angle3=%.2f", angle1, angle2, angle3);
}

void coordinateBolaCallback(const FP_Magang::coordinate::ConstPtr& msg) {
    if (basestatus != 1) {
        bx = msg->x;
        by = msg->y;
        // ROS_INFO("Received coordinates: x=%.2f, y=%.2f", bx, by);
    }
}

void coordinateRobotCallback(const FP_Magang::BS2PC::ConstPtr& msg) {
    tx = msg->tujuan_x;
    ty = msg->tujuan_y;
    // ROS_INFO("Received Robot coordinates: x=%.2f, y=%.2f", tx, ty);
}

void encoderSpeed(double left, double right) {
    rx = -left * cos(anglel) + right * cos(anglel);
    ry = left * sin(anglel) + right * sin(anglel);
    // ROS_INFO("Received coordinates: x=%.2f, y=%.2f", rx, ry);
}

void encoderCallback(const FP_Magang::BS2PC::ConstPtr& msg) {
    double left = msg->enc_left;
    double right = msg->enc_right;

    encoderSpeed(left, right);
}

void loopCallback(const ros::TimerEvent&) {
    switch (basestatus) {
    case 1111:
        status1();
        break;
    case 2222:
        status2();
        break;
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
        bx = 0;
        by = 0;
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
    coord_sub = nh.subscribe("/ball_coordinate", 50, coordinateBolaCallback);
    enc_sub = nh.subscribe("/bs2pc", 50, encoderCallback);
    robot_sub = nh.subscribe("/bs2pc", 50, coordinateRobotCallback);
    loop = nh.createTimer(ros::Duration(0.02), loopCallback);
    pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 50);

    ros::spin();
    return 0;
}
