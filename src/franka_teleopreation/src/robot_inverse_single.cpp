#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <Eigen/Eigen>
#include <assert.h>

#include <robot_msgs/omega.h>
#include <robot_msgs/ik.h>
#include <std_msgs/Float64MultiArray.h>

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
  ros::Subscriber sub;
  ros::Subscriber joint_sub;
  ros::Publisher pub;

  Eigen::Vector3d master_pos_zero;
  Eigen::Vector3d master_rpy_zero;
  Eigen::Vector3d master_pos;
  Eigen::Vector3d master_rpy;
  Eigen::Affine3d frame_end;

  Eigen::Vector3d slave_pos_zero;
  Eigen::Vector3d slave_desire_pos;
  Eigen::Matrix<double,3,3> slave_rotation_zero;
  Eigen::Matrix<double,3,3> slave_desire_rotation;

  Eigen::Vector3d slave_desire_rpy_r_increase;
  Eigen::Vector3d slave_desire_rpy_p_increase;
  Eigen::Vector3d slave_desire_rpy_y_increase;

  /* RCM*/
  Eigen::Vector3d postion_1;
  Eigen::Vector3d postion_2;
  Eigen::Vector3d postion_3;
  Eigen::Vector3d rotation_x_1;
  Eigen::Vector3d rotation_y_1;
  Eigen::Vector3d rotation_z_1;
  Eigen::Vector3d rotation_z_2_;
  Eigen::Vector3d rotation_z_2;
  Eigen::Vector3d rotation_x_2;
  Eigen::Vector3d rotation_y_2;
  Eigen::Vector3d rotation_axis;
  Eigen::Matrix<double,3,3> rotation_matrix;
  Eigen::Matrix<double,3,3> Identity_matrix;
  Eigen::Matrix<double,3,3> skew_matrix;
  double rotation_angle;


  Eigen::VectorXd joint_values;
  Eigen::VectorXd joint_status;

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

  double omega_button_zero;
  double omega_button;
  double omega_button_desire;
  bool omega_button_is_open;
  bool is_first_;

  std::string chain_start, chain_end, urdf_param;
  double timeout;
  const double error = 1e-5;
  unsigned int nj;
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  KDL::Vector p;
  KDL::Rotation M;
  KDL::Frame cartpos;
  double roll, pitch, yaw;

public:
  teleoperation():
    is_first_(true)
  {
    direction_pos_x = -1;
    direction_pos_y = 1;
    direction_pos_z = 1;
    direction_rpy_r = 1;
    direction_rpy_p = 1;
    direction_rpy_y = 1;
    scale_p_x = 0.4;
    scale_p_y = 0.4;
    scale_p_z = 0.4;
    scale_r_x = 0.05;
    scale_r_y = 0.05;
    scale_r_z = 0.05;
    joint_status.resize(7);
    postion_3 << 0.574298, -3.3812e-12, 0.564477;
    Identity_matrix = Eigen::Matrix3d::Identity(3,3);

    std::cout<<Identity_matrix<<std::endl;
    pub = nh.advertise<robot_msgs::ik>("/ik", 100, true);
    sub = nh.subscribe("/omega_pose", 100, &teleoperation::operationCallback, this);
    joint_sub = nh.subscribe("joint_pos_vector", 1, &teleoperation::jointstatusCallback, this);


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

    jointpositions(0) = 0;
    jointpositions(1) = 0;
    jointpositions(2) = 0;
    jointpositions(3) = -1.3;
    jointpositions(4) = 0;
    jointpositions(5) = 1.5;
    jointpositions(6) = 0.9;

//    for(unsigned int i=0; i< nj; i++){
//        jointpositions(i)= 0;
//    }

    bool kinematics_status;
    kinematics_status = fk_solver.JntToCart(jointpositions,cartpos);

    p = cartpos.p;   // Origin of the Frame
    M = cartpos.M; // Orientation of the Frame

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

  void jointstatusCallback(const std_msgs::Float64MultiArray::ConstPtr& jointpos_msg)
  {
    for(int i = 0; i < 7; i++)
    {
      joint_status[i] = jointpos_msg->data[i];
    }
//    std::cout << joint_status[0] <<std::endl;
  }

  void operationCallback(const robot_msgs::omega::ConstPtr& omega7_msg)
  {
    if(is_first_)
    {
      for(unsigned int i=0;i<3;i++)
          master_pos_zero[i] = omega7_msg->data[i];
      for(unsigned int i=0;i<3;i++)
          master_rpy_zero[i] = omega7_msg->data[i+3];
      omega_button_zero = omega7_msg->button[0];

      slave_pos_zero[0] = p[0];
      slave_pos_zero[1] = p[1];
      slave_pos_zero[2] = p[2];
      slave_rotation_zero(0,0) = M(0,0); slave_rotation_zero(0,1) = M(0,1); slave_rotation_zero(0,2) = M(0,2);
      slave_rotation_zero(1,0) = M(1,0); slave_rotation_zero(1,1) = M(1,1); slave_rotation_zero(1,2) = M(1,2);
      slave_rotation_zero(2,0) = M(2,0); slave_rotation_zero(2,1) = M(2,1); slave_rotation_zero(2,2) = M(2,2);


      std::cout <<slave_pos_zero<< std::endl;
      std::cout <<slave_rotation_zero<< std::endl;

      is_first_=false;
      return;
    }
    for(unsigned int i=0;i<3;i++)
        master_pos[i] = omega7_msg->data[i];
    for(unsigned int i=0;i<3;i++)
        master_rpy[i] = omega7_msg->data[i+3];
    omega_button = omega7_msg->button[0];

    omega_button_desire = omega_button - omega_button_zero;

    slave_desire_pos[0] = direction_pos_x * (master_pos[0]-master_pos_zero[0]) * scale_p_x + slave_pos_zero[0];
    slave_desire_pos[1] = direction_pos_y * (master_pos[1]-master_pos_zero[1]) * scale_p_y + slave_pos_zero[1];
    slave_desire_pos[2] = direction_pos_z * (master_pos[2]-master_pos_zero[2]) * scale_p_y + slave_pos_zero[2];

    slave_desire_rpy_r_increase[0] = direction_rpy_r * scale_r_x*(master_rpy[0]-master_rpy_zero[0]);
    slave_desire_rpy_p_increase[1] = direction_rpy_p * scale_r_y*(master_rpy[1]-master_rpy_zero[1]);
    slave_desire_rpy_y_increase[2] = direction_rpy_y * scale_r_z*(master_rpy[2]-master_rpy_zero[2]);

    slave_desire_rotation = (Eigen::AngleAxisd(slave_desire_rpy_y_increase[2],Eigen::Vector3d::UnitZ()))*
                            (Eigen::AngleAxisd(slave_desire_rpy_p_increase[1], Eigen::Vector3d::UnitY()))*
                            (Eigen::AngleAxisd(slave_desire_rpy_r_increase[0], Eigen::Vector3d::UnitX()))
                             *slave_rotation_zero;

    /* RCM algorithm */
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
    assert(nj == joint_status.size());

    KDL::JntArray jointpositions(nj);

    jointpositions(0) = joint_status[0];
    jointpositions(1) = joint_status[1];
    jointpositions(2) = joint_status[2];
    jointpositions(3) = joint_status[3];
    jointpositions(4) = joint_status[4];
    jointpositions(5) = joint_status[5];
    jointpositions(6) = joint_status[6];

    bool kinematics_status;
    KDL::Vector current_position;
    KDL::Rotation current_orientation;
    KDL::Frame current_position_orientation;

    kinematics_status = fk_solver.JntToCart(jointpositions, current_position_orientation);

    current_position = current_position_orientation.p;
    current_orientation = current_position_orientation.M;
    postion_1[0] = current_position(0);
    postion_1[1] = current_position(1);
    postion_1[2] = current_position(2);

    rotation_x_1[0] = current_orientation(0,0);
    rotation_x_1[1] = current_orientation(1,0);
    rotation_x_1[2] = current_orientation(2,0);

    rotation_y_1[0] = current_orientation(0,1);
    rotation_y_1[1] = current_orientation(1,1);
    rotation_y_1[2] = current_orientation(2,1);

    rotation_z_1[0] = current_orientation(0,2);
    rotation_z_1[1] = current_orientation(1,2);
    rotation_z_1[2] = current_orientation(2,2);

    postion_2 = slave_desire_pos;

    rotation_z_2 = postion_2 - postion_3;
    rotation_z_2_ = rotation_z_2;

    rotation_z_1.normalize();
    rotation_z_2_.normalize();

    rotation_axis = rotation_z_1.cross(rotation_z_2_);
    rotation_angle = std::acos(rotation_z_1.dot(rotation_z_2_));

    skew_matrix <<
                        0, -rotation_axis[2],  rotation_axis[1],
         rotation_axis[2],                0 , -rotation_axis[0],
        -rotation_axis[1],  rotation_axis[0],                0 ;

    rotation_matrix = std::cos(rotation_angle) * Identity_matrix +
        (1 - std::cos(rotation_angle)) * rotation_axis * rotation_axis.transpose() +
        std::sin(rotation_angle) * skew_matrix;

    rotation_x_2 = rotation_matrix * rotation_x_1;
    rotation_y_2 = rotation_matrix * rotation_y_1;

    KDL::Frame F_dest;

    F_dest.M(0,0) = rotation_x_2[0];
    F_dest.M(1,0) = rotation_x_2[1];
    F_dest.M(2,0) = rotation_x_2[2];

    F_dest.M(0,1) = rotation_y_2[0];
    F_dest.M(1,1) = rotation_y_2[1];
    F_dest.M(2,1) = rotation_y_2[2];

    F_dest.M(0,2) = rotation_z_2[0];
    F_dest.M(1,2) = rotation_z_2[1];
    F_dest.M(2,2) = rotation_z_2[2];
    /* RCM algorithm */

    /* NO RCM algorithm */
//    F_dest.M(0,0) = slave_desire_rotation(0,0);
//    F_dest.M(0,1) = slave_desire_rotation(0,1);
//    F_dest.M(0,2) = slave_desire_rotation(0,2);

//    F_dest.M(1,0) = slave_desire_rotation(1,0);
//    F_dest.M(1,1) = slave_desire_rotation(1,1);
//    F_dest.M(1,2) = slave_desire_rotation(1,2);

//    F_dest.M(2,0) = slave_desire_rotation(2,0);
//    F_dest.M(2,1) = slave_desire_rotation(2,1);
//    F_dest.M(2,2) = slave_desire_rotation(2,2);

    F_dest.p(0) = slave_desire_pos[0];
    F_dest.p(1) = slave_desire_pos[1];
    F_dest.p(2) = slave_desire_pos[2];

//    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error);

    KDL::JntArray joint_seed(nj);
    KDL::SetToZero(joint_seed);
    KDL::JntArray result(joint_seed);

    joint_seed(0) =  0;
    joint_seed(1) =  0;
    joint_seed(2) =  0;
    joint_seed(3) =  -1.3;
    joint_seed(4) =  0;
    joint_seed(5) =  1.5;
    joint_seed(6) =  0.9;

    int rc=ik_solver.CartToJnt(joint_seed, F_dest, result);
    if(rc < 0)
        printf("%s \n","Error: could not calculate forward kinematics");
    else{
//        printf("%s \n","TRAC IK Succes");
//        for(unsigned int i = 0; i < nj; i++)
//            std::cout << result(i) << " ";
    }

    robot_msgs::ik ik_msg;
    ik_msg.data.resize(7);
    joint_values.resize(7);

    for(int i = 0; i < 7; i++)
    {
      ik_msg.data[i] = result(i);
    }
    pub.publish(ik_msg);

//    std::cout << "ik_msg: " << std::endl<<ik_msg <<std::endl;

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































