#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <chrono>
#include "dhdc.h"

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <robot_msgs/omega.h>

// some platforms do not define M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FRE 300

//#define SAVE_DATA_

double pose_x =0;
double pose_y = 0;
double pose_z = 0;

double pose_x_r =0;
double pose_y_r = 0;
double pose_z_r = 0;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "master_read_pose");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh;
    ros::Rate rate(FRE);

    ros::Publisher pub_pose1=nh.advertise<robot_msgs::omega>("omega1/omega_map",100,true);
    ros::Publisher pub_pose2=nh.advertise<robot_msgs::omega>("omega2/omega_map",100,true);
    double rot_r = 0;
    double rot_p = 0;
    double rot_y =0;
    double gripper_angle=0;
    double rot_r_r = 0;
    double rot_p_r = 0;
    double rot_y_r =0;
    double gripper_angle_r=0;
    int done = 0;
    
    //get device count
    int device_count = dhdGetDeviceCount();
    if (device_count <= 0) {
        printf("error: %s\n", dhdErrorGetLastStr());
        ROS_ERROR("no device found");
        ros::shutdown();
        return 0;
    }
    if (device_count == 1) {
        ROS_ERROR("only one device found");
        ros::shutdown();
        return 0;
    }

    int id0 = dhdOpenID(0);
    if (id0 < 0) {
        printf("error: cannot open device 0(%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        ros::shutdown();
        return -1;
    }
    int id1 = dhdOpenID(1);
    if (id1 < 0) {
        printf("error: cannot open device 1(%s)\n", dhdErrorGetLastStr());
        dhdSleep(2.0);
        ros::shutdown();
        return -1;
    }
    printf("%s device detected\n", dhdGetSystemName(id0));
    printf("%s device detected\n", dhdGetSystemName(id1));
    // enable force
    dhdEnableForce(DHD_ON,id0);
    dhdEnableForce(DHD_ON,id1);
    while(ros::ok() && !done)
    {
        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, id0) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, id1) < DHD_NO_ERROR) {
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        
        dhdGetGripperAngleRad(&gripper_angle,id1);//0.0001-0.4921
//        gripper_angle*=-1;
        dhdGetPosition(&pose_x, &pose_y, &pose_z,id1);
        dhdGetOrientationRad(&rot_r, &rot_p, &rot_y,id1);

        dhdGetGripperAngleRad(&gripper_angle_r,id0);//0.0001-0.4921
        dhdGetPosition(&pose_x_r, &pose_y_r, &pose_z_r,id0);
        dhdGetOrientationRad(&rot_r_r, &rot_p_r, &rot_y_r,id0);

        robot_msgs::omega msg1,msg2;
        msg1.data.resize(6);
        msg1.button.resize(1);
        msg2.data.resize(6);
        msg2.button.resize(1);
        msg1.data[0] = pose_x;	  msg1.data[1] = pose_y;	  msg1.data[2] = pose_z;
        msg1.data[3] = rot_r;	    msg1.data[4] = rot_p;	    msg1.data[5] = rot_y;
        msg2.data[0] = pose_x_r;	msg2.data[1] = pose_y_r;	msg2.data[2] = pose_z_r;
        msg2.data[3] = rot_r_r;	  msg2.data[4] = rot_p_r;	  msg2.data[5] = rot_y_r;
        msg1.button[0] = gripper_angle;
        msg2.button[0] = gripper_angle_r;

        pub_pose1.publish(msg1);
        pub_pose2.publish(msg2);

        bool isRate = rate.sleep();
        std::cout<<msg1<<std::endl;
//        std::cout<<msg2<<std::endl;
//        if(!isRate)
//            std::cout<<"loop rate failed"<<std::endl;
    }

    dhdClose(id0);
    dhdClose(id1);
    std::cout<<"done.\n";
    ros::shutdown();

    return 0;

}


















