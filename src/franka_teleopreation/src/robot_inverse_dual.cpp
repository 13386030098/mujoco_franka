#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <stdio.h>
#include <robot_inverse.h>
#include <robot_msgs/omega.h>
#include <robot_msgs/ik.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class teleoperation
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_omega1;
  ros::Subscriber sub_omega2;
  ros::Publisher pub_omega1;
  ros::Publisher pub_omega2;

  HomoKinematics kinematics_1;
  HomoKinematics kinematics_2;

  Eigen::Vector3d master_1_pos_zero;
  Eigen::Vector3d master_2_pos_zero;
  Eigen::Vector3d master_1_rpy_zero;
  Eigen::Vector3d master_2_rpy_zero;
  Eigen::Vector3d master_1_pos;
  Eigen::Vector3d master_2_pos;
  Eigen::Vector3d master_1_rpy;
  Eigen::Vector3d master_2_rpy;

  Eigen::Affine3d frame_end_zero_position_1;
  Eigen::Affine3d frame_end_zero_position_2;

//  Eigen::Affine3d frame_end_zero_rotation_1;
//  Eigen::Affine3d frame_end_zero_rotation_2;

  Eigen::Affine3d frame_end_1;
  Eigen::Affine3d frame_end_2;

  Eigen::Vector3d slave_1_pos_zero;
  Eigen::Vector3d slave_2_pos_zero;

  Eigen::Vector3d slave_1_desire_pos;
  Eigen::Vector3d slave_2_desire_pos;

//  Eigen::Matrix<double,3,3> slave_desire_rotation;
//  Eigen::Matrix<double,3,3> slave_rotation_zero;
//  Eigen::Matrix<double,3,3> frame_end_constant;

  Eigen::Vector3d slave_1_desire_rpy_r_increase;
  Eigen::Vector3d slave_1_desire_rpy_p_increase;
  Eigen::Vector3d slave_1_desire_rpy_y_increase;

  Eigen::Vector3d slave_2_desire_rpy_r_increase;
  Eigen::Vector3d slave_2_desire_rpy_p_increase;
  Eigen::Vector3d slave_2_desire_rpy_y_increase;

  Eigen::VectorXd joint_values_omega_1;
  Eigen::VectorXd joint_values_omega_2;

  double direction_pos_x;
  double direction_pos_y;
  double direction_pos_z;
  double direction_rpy_r;
  double direction_rpy_p;
  double direction_rpy_y;
  double scale_p_x;
  double scale_p_y;
  double scale_p_z;
  double scale_r_x;
  double scale_r_y;
  double scale_r_z;
//  double yaw;
//  double pitch;
//  double roll;
  double rotate_angle_1;
  double rotate_angle_2;
  double roll_angle_1;
  double roll_angle_2;
  double clip_angle_1;
  double clip_angle_2;

  double omega_1_button_zero;
  double omega_2_button_zero;
  double omega_1_button;
  double omega_2_button;

  double omega_1_button_desire;
  double omega_2_button_desire;


  bool is_first_1;
  bool is_first_2;

public:
  teleoperation():
    is_first_1(true),
    is_first_2(true)
  {
    direction_pos_x = -1;
    direction_pos_y = 1;
    direction_pos_z = 1;
    direction_rpy_r = 1;
    direction_rpy_p = 1;
    direction_rpy_y = 1;
    scale_p_x = 0.3;
    scale_p_y = 0.3;
    scale_p_z = 0.3;
    scale_r_x = 0.6;
    scale_r_y = 0.6;
    scale_r_z = 0.6;

    std::cout<<"teleoperation start ..."<<std::endl;
    pub_omega1 = nh.advertise<robot_msgs::ik>("omega1/ik", 100, true);
    pub_omega2 = nh.advertise<robot_msgs::ik>("omega2/ik", 100, true);
    sub_omega1 = nh.subscribe("omega1/omega_map", 100, &teleoperation::operationCallback_1, this);
    sub_omega2 = nh.subscribe("omega2/omega_map", 100, &teleoperation::operationCallback_2, this);
  }

  ~teleoperation(){}

  void operationCallback_1(const robot_msgs::omega::ConstPtr& omega7_msg)
  {
    if(is_first_1)
    {
      for(unsigned int i=0;i<3;i++)
          master_1_pos_zero[i] = omega7_msg->data[i];
      for(unsigned int i=0;i<3;i++)
          master_1_rpy_zero[i] = omega7_msg->data[i+3];

      omega_1_button_zero = omega7_msg->button[0];
      kinematics_1.getTransformAtIndex(10, frame_end_zero_position_1);
//      kinematics_.getTransformAtIndex_rotation(11, frame_end_zero_rotation_1);
      slave_1_pos_zero = frame_end_zero_position_1.translation();
//      slave_rotation_zero = frame_end_zero_rotation_1.rotation();
      is_first_1=false;
      return;
    }
    for(unsigned int i=0;i<3;i++)
        master_1_pos[i] = omega7_msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master_1_rpy[i] = omega7_msg->data[i+3];
    omega_1_button = omega7_msg->button[0];

    omega_1_button_desire = omega_1_button - omega_1_button_zero;

    slave_1_desire_pos[0] = direction_pos_x * (master_1_pos[0]-master_1_pos_zero[0]) * scale_p_x + slave_1_pos_zero[0];
    slave_1_desire_pos[1] = direction_pos_y * (master_1_pos[1]-master_1_pos_zero[1]) * scale_p_y + slave_1_pos_zero[1];
    slave_1_desire_pos[2] = direction_pos_z * (master_1_pos[2]-master_1_pos_zero[2]) * scale_p_y + slave_1_pos_zero[2];

    slave_1_desire_rpy_r_increase[0] = direction_rpy_r * scale_r_x*(master_1_rpy[0]-master_1_rpy_zero[0]);
    slave_1_desire_rpy_p_increase[1] = direction_rpy_p * scale_r_y*(master_1_rpy[1]-master_1_rpy_zero[1]);
    slave_1_desire_rpy_y_increase[2] = direction_rpy_y * scale_r_z*(master_1_rpy[2]-master_1_rpy_zero[2]);

//    roll   = slave_desire_rpy_r_increase[0];
//    pitch  = slave_desire_rpy_p_increase[1];
//    yaw    = slave_desire_rpy_y_increase[2];

//    slave_desire_rotation = (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()))*(Eigen::AngleAxisd(pitch,
//                                   Eigen::Vector3d::UnitY()))*(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
//                                 *slave_rotation_zero;

    frame_end_1 = Eigen::Translation3d(slave_1_desire_pos);

    joint_values_omega_1.resize(6);
    kinematics_1.getIk(frame_end_1, joint_values_omega_1);

//    frame_end_constant = slave_desire_rotation.inverse();

//    double rotate_angle = std::atan2(frame_end_constant(2,0), frame_end_constant(2,1));
//    double roll_angle = std::atan2((frame_end_constant(2,1) * std::cos(rotate_angle) - frame_end_constant(2,0)*std::sin(rotate_angle)), frame_end_constant(2,2));
//    double clip_angle = std::acos(frame_end_constant(0,0) * std::cos(rotate_angle) + frame_end_constant(0,1)*std::sin(rotate_angle));

    rotate_angle_1 = slave_1_desire_rpy_r_increase[0];
    roll_angle_1   = slave_1_desire_rpy_p_increase[1];
    clip_angle_1   = slave_1_desire_rpy_y_increase[2];

    joint_values_omega_1[3] = clip_angle_1;
    joint_values_omega_1[4] = rotate_angle_1;
    joint_values_omega_1[5] = roll_angle_1;

    robot_msgs::ik ik_msg;
    ik_msg.data.resize(6);
    ik_msg.data[0] = joint_values_omega_1[0];
    ik_msg.data[1] = joint_values_omega_1[1];
    ik_msg.data[2] = joint_values_omega_1[2];
    ik_msg.data[3] = joint_values_omega_1[3];
    ik_msg.data[4] = joint_values_omega_1[4];
//    ik_msg.data[5] = joint_values_omega_1[5];
    ik_msg.data[5] = omega_1_button_desire;

    pub_omega1.publish(ik_msg);

    std::cout <<ik_msg <<std::endl;

  }

  void operationCallback_2(const robot_msgs::omega::ConstPtr& omega7_msg)
  {
    if(is_first_2)
    {
      for(unsigned int i=0;i<3;i++)
          master_2_pos_zero[i] = omega7_msg->data[i];
      for(unsigned int i=0;i<3;i++)
          master_2_rpy_zero[i] = omega7_msg->data[i+3];

      omega_2_button_zero = omega7_msg->button[0];
      kinematics_2.getTransformAtIndex(10, frame_end_zero_position_2);
//      kinematics_.getTransformAtIndex_rotation(11, frame_end_zero_rotation_2);
      slave_2_pos_zero = frame_end_zero_position_2.translation();
//      slave_rotation_zero = frame_end_zero_rotation_2.rotation();
      is_first_2=false;
      return;
    }
    for(unsigned int i=0;i<3;i++)
        master_2_pos[i] = omega7_msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master_2_rpy[i] = omega7_msg->data[i+3];
    omega_2_button = omega7_msg->button[0];
    omega_2_button_desire = omega_2_button - omega_2_button_zero;


    slave_2_desire_pos[0] = direction_pos_x * (master_2_pos[0]-master_2_pos_zero[0]) * scale_p_x + slave_2_pos_zero[0];
    slave_2_desire_pos[1] = direction_pos_y * (master_2_pos[1]-master_2_pos_zero[1]) * scale_p_y + slave_2_pos_zero[1];
    slave_2_desire_pos[2] = direction_pos_z * (master_2_pos[2]-master_2_pos_zero[2]) * scale_p_y + slave_2_pos_zero[2];

    slave_2_desire_rpy_r_increase[0] = direction_rpy_r * scale_r_x*(master_2_rpy[0]-master_2_rpy_zero[0]);
    slave_2_desire_rpy_p_increase[1] = direction_rpy_p * scale_r_y*(master_2_rpy[1]-master_2_rpy_zero[1]);
    slave_2_desire_rpy_y_increase[2] = direction_rpy_y * scale_r_z*(master_2_rpy[2]-master_2_rpy_zero[2]);

//    roll   = slave_desire_rpy_r_increase[0];
//    pitch  = slave_desire_rpy_p_increase[1];
//    yaw    = slave_desire_rpy_y_increase[2];

//    slave_desire_rotation = (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()))*(Eigen::AngleAxisd(pitch,
//                                   Eigen::Vector3d::UnitY()))*(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
//                                 *slave_rotation_zero;

    frame_end_2 = Eigen::Translation3d(slave_2_desire_pos);

    joint_values_omega_2.resize(6);
    kinematics_2.getIk(frame_end_2, joint_values_omega_2);

//    frame_end_constant = slave_desire_rotation.inverse();

//    double rotate_angle = std::atan2(frame_end_constant(2,0), frame_end_constant(2,1));
//    double roll_angle = std::atan2((frame_end_constant(2,1) * std::cos(rotate_angle) - frame_end_constant(2,0)*std::sin(rotate_angle)), frame_end_constant(2,2));
//    double clip_angle = std::acos(frame_end_constant(0,0) * std::cos(rotate_angle) + frame_end_constant(0,1)*std::sin(rotate_angle));

    rotate_angle_2 = slave_2_desire_rpy_r_increase[0];
    roll_angle_2   = slave_2_desire_rpy_p_increase[1];
    clip_angle_2   = slave_2_desire_rpy_y_increase[2];

    joint_values_omega_2[3] = clip_angle_2;
    joint_values_omega_2[4] = rotate_angle_2;
    joint_values_omega_2[5] = roll_angle_2;

    robot_msgs::ik ik_msg;
    ik_msg.data.resize(6);
    ik_msg.data[0] = joint_values_omega_2[0];
    ik_msg.data[1] = joint_values_omega_2[1];
    ik_msg.data[2] = joint_values_omega_2[2];
    ik_msg.data[3] = joint_values_omega_2[3];
    ik_msg.data[4] = joint_values_omega_2[4];
    ik_msg.data[5] = omega_2_button_desire;

    pub_omega2.publish(ik_msg);

    std::cout << ik_msg <<std::endl;

  }

  void start(void)
  {
      std::cout<<"remote operation start ..."<<std::endl;
  }
};


int main(int argc, char **argv)
{
    ros::init (argc, argv, "robot_inverse");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    teleoperation operation;
    operation.start();
    std::cout << "ok" << std::endl;
    ros::waitForShutdown();
    return 0;
}































