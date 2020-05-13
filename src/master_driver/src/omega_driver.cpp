#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dhdc.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <robot_msgs/omega.h>
#include <Eigen/Eigen>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define _USE_MATH_DEFINES
#define FRE  300   // 0.1sec

using namespace std;

int main(int  argc, char **argv)
{
    ros::init (argc, argv, "read_pose");
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    ros::NodeHandle nh;
    ros::Rate loop_rate(FRE);
    cout << "ok" <<endl;
    ros::Publisher pub_pose=nh.advertise<robot_msgs::omega>("/omega_pose",100,true);
    double pose_x =0, pose_y = 0, pose_z = 0;
    double rot_r = 0, rot_p = 0, rot_y =0;
    double gripper_angle=0;
    int done = 0;

    Eigen::Matrix<double,3,3> modify_matrix;
    modify_matrix(0,0) = -1;	modify_matrix(0,1) = 0;		modify_matrix(0,2) = 0;
    modify_matrix(1,0) = 0;	modify_matrix(1,1) = 1;	modify_matrix(1,2) = 0;
    modify_matrix(2,0) = 0;	modify_matrix(2,1) = 0;		modify_matrix(2,2) = 1;

    //get device count
    if (dhdGetDeviceCount() <= 0) {
        printf("error: %s\n", dhdErrorGetLastStr());
        ros::shutdown();
        return 0;
    }
    // open the first available device
    if (dhdOpen () < 0) {
      printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
      dhdSleep (2.0);
      ros::shutdown();
      return -1;
    }
    // identify device
    printf ("%s device detected\n\n", dhdGetSystemName());
    // enable force
    cout << "ok" <<endl;

    dhdEnableForce(DHD_ON);   // cout<< "ok"<<endl;
   // cout << "ok" <<endl;

    while(ros::ok() && !done)
    {
        // apply zero force
        if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR)
        {
          printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
          done = 1;
        }
        if (dhdGetPosition(&pose_x, &pose_y, &pose_z) < DHD_NO_ERROR)
        {
            printf("error: cannot read position (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        if (dhdGetOrientationRad(&rot_r, &rot_p, &rot_y) < DHD_NO_ERROR)
        {
            printf("error: cannot read position (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }
        if (dhdGetGripperAngleRad(&gripper_angle) < DHD_NO_ERROR)
        {
            printf("error: cannot read position (%s)\n", dhdErrorGetLastStr());
            done = 1;
        }

      Eigen::Matrix<double,3,1> position_orign,position_modify;
      position_orign(0,0) = pose_x;	position_orign(1,0) = pose_y;	position_orign(2,0) = pose_z;
      position_modify = modify_matrix * position_orign;

      robot_msgs::omega msg;
      msg.data.resize(6);
      msg.button.resize(1);
      msg.data[0] = position_modify(0,0);	msg.data[1] = position_modify(1,0);	msg.data[2] = position_modify(2,0);
      msg.data[3] = rot_r;	              msg.data[4] = rot_p;	                msg.data[5] = rot_y;
      msg.button[0] = gripper_angle;
      pub_pose.publish(msg);
//      cout<< "msg.data[0]"<<msg.data[3]<<endl;
      cout<< "msg.data[1]"<<msg<<endl;
      loop_rate.sleep();
    }
    // close the connection
    dhdClose ();
    // happily exit
    printf ("\ndone.\n");
    ros::shutdown();
    return 0;
}































































