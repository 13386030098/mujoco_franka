#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <stdio.h>
#include <robot_msgs/omega.h>
#include <robot_msgs/ik.h>

#include <trac_ik/trac_ik.hpp>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>



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


  Eigen::Affine3d frame_end_1;
  Eigen::Affine3d frame_end_2;

  Eigen::Vector3d slave_1_pos_zero;
  Eigen::Vector3d slave_2_pos_zero;

  Eigen::Vector3d slave_1_desire_pos;
  Eigen::Vector3d slave_2_desire_pos;

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

  std::string chain_start, chain_end, urdf_param;
  double timeout;
  const double error = 1e-5;
  unsigned int nj;
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  KDL::Vector p;
  KDL::Rotation M;

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

    nh.param("chain_start", chain_start, std::string("panda_link0"));
    nh.param("chain_end", chain_end, std::string("panda_link8"));

    if (chain_start=="" || chain_end=="") {
        ROS_FATAL("Missing chain info in launch file");
        exit (-1);
    }
    nh.param("timeout", timeout, 0.005);
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));

    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error);
    bool valid = ik_solver.getKDLChain(chain);
    if (!valid){
        ROS_ERROR("There was no valid KDL chain found");
        exit (-1);
    }
    valid = ik_solver.getKDLLimits(ll,ul);
    if (!valid){
        ROS_INFO("There were no valid KDL joint limits found");
        exit (-1);
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain);

    nj = chain.getNrOfJoints();
    ROS_INFO ("Using %d joints", nj);
    KDL::JntArray jointpositions(nj);

    for(unsigned int i=0; i< nj; i++){
        jointpositions(i)= 0;
    }

    KDL::Frame cartpos;

    bool kinematics_status;
    kinematics_status = fk_solver.JntToCart(jointpositions,cartpos);

    p = cartpos.p;   // Origin of the Frame
    M = cartpos.M; // Orientation of the Frame

    double roll, pitch, yaw;
    M.GetRPY(roll,pitch,yaw);

    if(kinematics_status>=0){
        printf("%s \n","KDL FK Succes");
        std::cout <<"Origin: " << p(0) << "," << p(1) << "," << p(2) << std::endl;
        std::cout <<"RPY: " << roll << "," << pitch << "," << yaw << std::endl;

     }else{
         printf("%s \n","Error: could not calculate forward kinematics :(");
    }

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































