////TETRA_DS Service ROS Package_Ver 0.1
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp" //teb_poses...
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker.hpp" //teb markers..
#include "sensor_msgs/msg/point_cloud2.hpp" //bumper
#include "sensor_msgs/msg/point_field.hpp" //bumper
#include "sensor_msgs/msg/joy.hpp" //add 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

//Service
#include "tetra_msgs/msg/obstacles.hpp"
#include "tetra_msgs/msg/obstacles2.hpp"
#include "tetra_msgs/srv/getlocation.hpp" //SRV
#include "tetra_msgs/srv/gotolocation.hpp" //SRV
#include "tetra_msgs/srv/gotolocation2.hpp" //SRV
#include "tetra_msgs/srv/setlocation.hpp" //SRV
#include "tetra_msgs/srv/setsavemap.hpp" //SRV
#include "tetra_msgs/srv/getinformation.hpp" //SRV
#include "tetra_msgs/srv/dockingcontrol.hpp" //SRV
#include "tetra_msgs/srv/getlocationlist.hpp" //SRV
#include "tetra_msgs/srv/getlandmarklist.hpp" //SRV
#include "tetra_msgs/srv/deletelocation.hpp" //SRV
#include "tetra_msgs/srv/deletelandmark.hpp" //SRV
#include "tetra_msgs/srv/runmapping.hpp" //SRV
#include "tetra_msgs/srv/runnavigation.hpp" //SRV
#include "tetra_msgs/srv/rosnodekill.hpp" //SRV
#include "tetra_msgs/srv/getmaplist.hpp" //SRV
#include "tetra_msgs/srv/deletemap.hpp" //SRV
#include "tetra_msgs/srv/ledcontrol.hpp" //SRV
#include "tetra_msgs/srv/ledtogglecontrol.hpp" //SRV
#include "tetra_msgs/srv/toggleon.hpp" //SRV
#include "tetra_msgs/srv/gotocancel.hpp" //SRV
#include "tetra_msgs/srv/setmaxspeed.hpp" //SRV
#include "tetra_msgs/srv/accelerationslop.hpp" //SRV
#include "tetra_msgs/srv/servo.hpp" //SRV
#include "ar_track_alvar_msgs/msg/alvar_markers.hpp" //MSG AR_TAG
#include "tetra_msgs/srv/setinitpose.hpp" //SRV
#include "tetra_msgs/srv/virtual_obstacle.hpp" //SRV
#include "tetra_msgs/srv/pose_estimate.hpp" //SRV
//Conveyor Service
#include "tetra_msgs/srv/gotoconveyor.hpp" //SRV
#include "tetra_msgs/srv/loadingcheck.hpp" //SRV
#include "tetra_msgs/srv/unloadingcheck.hpp" //SRV
//add patrol service//
#include "tetra_msgs/srv/patrol.hpp" //SRV
#include "tetra_msgs/srv/patrol_conveyor.hpp" //SRV
#include "tetra_msgs/srv/conveyor_auto_movement.h" //SRV
//add delete data all service//
#include "tetra_msgs/srv/deletedataall.hpp" //SRV
//IMU Service//
#include "tetra_msgs/srv/all_data_reset.hpp"
#include "tetra_msgs/srv/euler_angle_init.hpp"
#include "tetra_msgs/srv/euler_angle_reset.hpp"
#include "tetra_msgs/srv/pose_velocity_reset.hpp"
#include "tetra_msgs/srv/reboot_sensor.hpp"
//robot_localization//
#include "tetra_msgs/srv/set_pose.hpp"
//Home ID Set Service//
#include "tetra_msgs/srv/sethome_id.hpp"
//Set EKF & IMU Reset Service//
#include "tetra_msgs/srv/setekf.hpp"
//SONAR Sensor On/Off
#include "tetra_msgs/srv/power_sonar_cmd.hpp" //SRV

#include <sstream>
#include <vector>
#include <chrono>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <thread> //thread add...
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/types.h>
#include <dirent.h>
#include <error.h>
#include <cstdlib> //std::system call
#include <algorithm>
#include <signal.h>

#define LOW_BATTERY 15
#define MAX_RETRY_CNT 999
#define BUF_LEN 4096
using namespace std;
using namespace std::chrono_literals;
std::string tf_prefix_;
string m_strRobotIP = "";
int m_iRotation_Mode = 0; //Docking Rotation Mode Select
int m_iReset_flag = 0;
FILE *fp;
int status;
char Textbuffer[BUF_LEN];

//test... Marker Tracking & Front docking...
int EX_MARKER_ID = 0;
int m_iParticleCloud_size = 0;
pthread_t p_docking_thread;
pthread_t p_auto_thread;
int  m_iRetry_cnt = 0;
//Docking Command //
int ex_iDocking_CommandMode = 0;
int m_iDocking_timeout_cnt = 0;
int m_iNoMarker_cnt = 0;
//Teb Pose Info
double m_dTeb_Pose_head_Angle[1024] = {0.0, };
//patrol...//
int  m_patrol_location_cnt = 0;
string arr_patrol_location[255] = {"", };
//reset Timer count../
int  m_iTimer_cnt = 0;
// Active map Check //
bool m_bActive_map_check = false;

double m_dTF_Yaw = 0.0;
double m_dTF_New_Pose_X = 0.0;
double m_dTF_New_Pose_Y = 0.0;
int m_iList_Count = 0;
int m_iList_Count2 = 0;
int m_iMode_Count = 0;
int m_iMode_Count2 = 0;
//Teb Via Point...
int m_iViaPoint_Index = 0;
bool m_bFlag_nomotion_call = false;
//clesr_costmap service call...
bool m_flag_clesr_costmap_call = false;
//Dynamic_reconfigure call flag//
bool m_flag_Dynamic_reconfigure_call = false;
bool m_flag_Dynamic_TebMarkers_major_update = false;
bool m_flag_Dynamic_Teblocalplan_major_update = false;
bool m_flag_Dynamic_Teblocalplan_minor_update = false;
bool m_flag_Dynamic_Linear_velocity_major_update = false;
bool m_flag_Dynamic_Linear_velocity_minor_update = false;
//Set Goal flag//
bool m_flag_setgoal = false;
//flag
bool m_flag_PREEMPTED = false;


typedef struct HOME_POSE
{
    string HOME_strLOCATION = "HOME";
    double HOME_dPOSITION_X = 0.6;
    double HOME_dPOSITION_Y = 0.0;
    double HOME_dPOSITION_Z = 0.0;
    double HOME_dQUATERNION_X = 0.0;
    double HOME_dQUATERNION_Y = 0.0;
    double HOME_dQUATERNION_Z = 0.0;
    double HOME_dQUATERNION_W = 1.0;

}HOME_POSE;
HOME_POSE _pHomePose;

typedef struct LANDMARK_POSE
{
    string header_frame_id;
    string ns;
    int mark_id;
    double pose_position_x;
    double pose_position_y;
    double pose_position_z;
    double pose_orientation_x;
    double pose_orientation_y;
    double pose_orientation_z;
    double pose_orientation_w;

}LANDMARK_POSE;

typedef struct FALG_VALUE
{
    bool m_bflag_NextStep = false;
    bool m_bflag_ComebackHome = false;
    bool m_bfalg_DockingExit = false;
    bool m_bflag_Conveyor_docking = false;
    bool m_bFlag_Disable_bumper = false;
    bool m_Onetime_reset_flag = false;
    //Tracking obstacle Check//
    bool m_bFlag_Obstacle_Right = false;
    bool m_bFlag_Obstacle_Center = false;
    bool m_bFlag_Obstacle_Left = false;
    //PCL obstacle Check//
    bool m_bFlag_Obstacle_PCL1 = false;
    bool m_bFlag_Obstacle_PCL2 = false;
    //Error Flag//
    bool m_bumperhit_flag = false;
    bool m_emgpush_flag = false;
    bool BUMPER_BT = false;
    //Dynamic reconfigure flag//
    bool m_bTebMarker_reconfigure_flag = false;
    bool m_bflag_patrol = false;
    bool m_bflag_patrol2 = false;
    bool m_bflag_goto_cancel = false;
    bool m_bflagGo = false;
    bool m_bflagGo2 = false;
    bool m_bCorneringFlag = true;
    //no motion service call flag//
    bool m_bFlag_nomotion = true;
    bool m_bFlag_Initialpose = false;
    //Setgoal_pub flag//
    bool m_bFlag_pub = false;

}FALG_VALUE;
FALG_VALUE _pFlag_Value;

//landmark...
typedef struct LAND_MARK_POSE
{
    float init_position_x = 0.0;
    float init_position_y = 0.0;
    float init_position_z = 0.0;
    float init_orientation_z = 0.0;
    float init_orientation_w = 1.0;

}LAND_MARK_POSE;
LAND_MARK_POSE _pLandMarkPose;

//amcl_Pose...
typedef struct AMCL_POSE
{
    double poseAMCLx = 0.0;
    double poseAMCLy = 0.0;
    double poseAMCLz = 0.0;
    double poseAMCLqx = 0.0;
    double poseAMCLqy = 0.0;
    double poseAMCLqz = 0.0;
    double poseAMCLqw = 0.0;

}AMCL_POSE;
AMCL_POSE _pAMCL_pose;

//tf_Pose.(map->base_footprint TF)..
typedef struct TF_POSE
{
    double poseTFx = 0.0;
    double poseTFy = 0.0;
    double poseTFz = 0.0;
    double poseTFqx = 0.0;
    double poseTFqy = 0.0;
    double poseTFqz = 0.0;
    double poseTFqw = 1.0;

}TF_POSE;
TF_POSE _pTF_pose;

//tf_Pose.(map->odom TF)..
typedef struct TF_POSE2
{
    double poseTFx2 = 0.0;
    double poseTFy2 = 0.0;
    double poseTFz2 = 0.0;
    double poseTFqx2 = 0.0;
    double poseTFqy2 = 0.0;
    double poseTFqz2 = 0.0;
    double poseTFqw2 = 1.0;

}TF_POSE2;
TF_POSE2 _pTF_pose2;

//goal_pose...
typedef struct GOAL_POSE
{
    float goal_positionX = 0.0;
    float goal_positionY = 0.0;
    float goal_positionZ = 0.0;
    float goal_quarterX = 0.0;
    float goal_quarterY = 0.0;
    float goal_quarterZ = 0.0;
    float goal_quarterW = 1.0;

}GOAL_POSE;
GOAL_POSE _pGoal_pose;

//Callback Value
typedef struct ROBOT_STATUS
{
    int m_iCallback_Battery = 0;
    int m_iCallback_ErrorCode = 0;
    int m_iCallback_EMG = 0;
    int m_iCallback_Bumper = 0;
    int m_iCallback_Charging_status = 0;
    //Bumper Collision Behavior//
    int m_iBumperCollisionBehavor_cnt = 0;
    //auto test
    int m_iMovebase_Result = 0;
    //Conveyor Info..(Option)
    double m_dLoadcell_weight = 0.0;
    int m_iConveyor_Sensor_info = 0;
    int HOME_ID = 0; //Docking ID Param Read//
    int CONVEYOR_ID = 0;
    int CONVEYOR_MOVEMENT = 0; // 0: nomal , 1: Loading , 2: Unloading

}ROBOT_STATUS;
ROBOT_STATUS _pRobot_Status;

//AR_TAG Pose
typedef struct AR_TAG_POSE
{
    int m_iSelect_AR_tag_id = 0;
    int m_iAR_tag_id_Index = 0;
    int m_iAR_tag_id = -1;
    float m_fAR_tag_pose_x = 0.0;
    float m_fAR_tag_pose_y = 0.0;
    float m_fAR_tag_pose_z = 0.0;
    float m_fAR_tag_orientation_x = 0.0;
    float m_fAR_tag_orientation_y = 0.0;
    float m_fAR_tag_orientation_z = 0.0;
    float m_fAR_tag_orientation_w = 0.0;
    double m_fAR_tag_roll = 0.0;
    double m_fAR_tag_pitch = 0.0;
    double m_fAR_tag_yaw = 0.0;
    //Transform AR Tag Axis -> Robot Axis
    float m_transform_pose_x = 0.0;
    float m_transform_pose_y = 0.0;
    float m_fPositioning_Angle = 0.0;
    //Calc Odom to AR_Marker TF
    double m_target_yaw = 0.0;

}AR_TAG_POSE;
AR_TAG_POSE _pAR_tag_pose;

// Ultrasonic_range//
float m_Ultrasonic_DL_Range = 0.0;
float m_Ultrasonic_DR_Range = 0.0;
float m_Ultrasonic_RL_Range = 0.0;
float m_Ultrasonic_RR_Range = 0.0;
//roslaunch mode check//
int ex_ilaunchMode = 0;

//dynamic parameter//
typedef struct DYNAMIC_PARAM
{
    double MAX_Linear_velocity = 0.0;
    double m_linear_vel = 0.0;
    double m_angular_vel = 0.0;
    double m_dweight_kinematics_forward_drive_default = 600.0;
    double m_dweight_kinematics_forward_drive_backward = 10.0;
}DYNAMIC_PARAM;
DYNAMIC_PARAM _pDynamic_param;

//SetEKF_Command Service 
typedef struct RESET_SRV
{
    bool   bflag_reset = false;
    double init_position_x = 0.0;
    double init_position_y = 0.0;
    double init_position_z = 0.0;
    double init_orientation_x = 0.0;
    double init_orientation_y = 0.0;
    double init_orientation_z = 0.0;
    double init_orientation_w = 1.0;

}RESET_SRV;
RESET_SRV _pReset_srv;

//Publisher & msg define....
geometry_msgs::msg::Twist cmd;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdpub_;
//TODO movebase -> nav2///////////
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr service_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr PoseReset_pub;
rclcpp::Publisher<actionlib_msgs::msg::GoalID>::SharedPtr GotoCancel_pub;
//////////////////////////////////////
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Accel_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo_pub;
geometry_msgs::msg::PoseWithCovarianceStamped initPose_;
std_msgs::msg::Int32 accel_vel;
std_msgs::msg::Int32 servo_request;
geometry_msgs::msg::Point goal_position;
geometry_msgs::msg::Quaternion goal_quarter;
//TODO///////////////////////////////////
move_base_msgs::msg::MoveBaseActionGoal goal;
///////////////////////////////////
//Docking_progress
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docking_progress_pub;
std_msgs::msg::Int32 docking_progress;

//Virtual Costmap//
rclcpp::Publisher<tetra_msgs::msg::Obstacles>::SharedPtr virtual_obstacle_pub;
//custom_msgs::Obstacles virtual_obstacle;
tetra_msgs::msg::Obstacles virtual_obstacle;
//Virtual Costmap2//
rclcpp::Publisher<tetra_msgs::msg::Obstacles2>::SharedPtr virtual_obstacle2_pub;
//custom_msgs::Obstacles virtual_obstacle2;
tetra_msgs::msg::Obstacles2 virtual_obstacle2;

// Docking positioning//
rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr positioning_pub;
geometry_msgs::msg::Pose2D positioning_pose;

//**Command srv _ Service Server************************/
tetra_msgs::srv::Getlocation getlocation_cmd;
rclcpp::Service<tetra_msgs::srv::Getlocation>::SharedPtr getlocation_service;
tetra_msgs::srv::Gotolocation goto_cmd;
rclcpp::Service<tetra_msgs::srv::Gotolocation>::SharedPtr goto_service;
tetra_msgs::srv::Gotolocation2 goto_cmd2;
rclcpp::Service<tetra_msgs::srv::Gotolocation2>::SharedPtr goto_service2;
tetra_msgs::srv::Setlocation setlocation_cmd;
rclcpp::Service<tetra_msgs::srv::Setlocation>::SharedPtr setlocation_service;
tetra_msgs::srv::Setsavemap savemap_cmd;
rclcpp::Service<tetra_msgs::srv::Setsavemap>::SharedPtr save_map_service;
tetra_msgs::srv::Getinformation getinfo_cmd;
rclcpp::Service<tetra_msgs::srv::Getinformation>::SharedPtr getinfo_service;
tetra_msgs::srv::Dockingcontrol docking_cmd;
rclcpp::Service<tetra_msgs::srv::Dockingcontrol>::SharedPtr docking_service;
tetra_msgs::srv::Getlocationlist locationlist_cmd;
rclcpp::Service<tetra_msgs::srv::Getlocationlist>::SharedPtr locationlist_service;
tetra_msgs::srv::Getlandmarklist landmarklist_cmd;
rclcpp::Service<tetra_msgs::srv::Getlandmarklist>::SharedPtr landmarklist_service;
tetra_msgs::srv::Deletelocation delete_location_cmd;
rclcpp::Service<tetra_msgs::srv::Deletelocation>::SharedPtr delete_location_service;
tetra_msgs::srv::Deletelandmark delete_landmark_cmd;
rclcpp::Service<tetra_msgs::srv::Deletelandmark>::SharedPtr delete_landmark_service;
tetra_msgs::srv::Deletemap delete_map_cmd;
rclcpp::Service<tetra_msgs::srv::Deletemap>::SharedPtr delete_map_service;
tetra_msgs::srv::Runmapping mapping_cmd;
rclcpp::Service<tetra_msgs::srv::Runmapping>::SharedPtr mapping_service;
tetra_msgs::srv::Runnavigation navigation_cmd;
rclcpp::Service<tetra_msgs::srv::Runnavigation>::SharedPtr navigation_service;
tetra_msgs::srv::Rosnodekill nodekill_cmd;
rclcpp::Service<tetra_msgs::srv::Rosnodekill>::SharedPtr nodekill_service;
tetra_msgs::srv::Getmaplist maplist_cmd;
rclcpp::Service<tetra_msgs::srv::Getmaplist>::SharedPtr maplist_service;
tetra_msgs::srv::Gotocancel gotocancel_cmd;
rclcpp::Service<tetra_msgs::srv::Gotocancel>::SharedPtr gotocancel_service;
tetra_msgs::srv::Setmaxspeed setspeed_cmd;
rclcpp::Service<tetra_msgs::srv::Setmaxspeed>::SharedPtr setspeed_service;
tetra_msgs::srv::Accelerationslop sloptime_cmd;
rclcpp::Service<tetra_msgs::srv::Accelerationslop>::SharedPtr sloptime_service;
tetra_msgs::srv::Servo servo_cmd;
rclcpp::Service<tetra_msgs::srv::Servo>::SharedPtr servo_service;
//virtual_costmap//
tetra_msgs::srv::VirtualObstacle virtual_obstacle_cmd;
rclcpp::Service<tetra_msgs::srv::VirtualObstacle>::SharedPtr virtual_obstacle_service;

//Set InitPose//
tetra_msgs::srv::Setinitpose setinitpose_cmd;
rclcpp::Service<tetra_msgs::srv::Setinitpose>::SharedPtr setinitpose_service;
//2D_Pose_Estimate//
tetra_msgs::srv::PoseEstimate pose_estimate_cmd;
rclcpp::Service<tetra_msgs::srv::PoseEstimate>::SharedPtr pose_Estimate_service;
//Docking Exit Service//
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr docking_exit;
std_srvs::srv::Empty m_request_dockiong_exit;
//Conveyor Service//
tetra_msgs::srv::Gotoconveyor gotoconveyor_cmd;
rclcpp::Service<tetra_msgs::srv::Gotoconveyor>::SharedPtr gotoconveyor_service;
tetra_msgs::srv::Loadingcheck loadingcheck_cmd;
rclcpp::Service<tetra_msgs::srv::Loadingcheck>::SharedPtr loadingcheck_service;
tetra_msgs::srv::Unloadingcheck unloadingcheck_cmd;
rclcpp::Service<tetra_msgs::srv::Unloadingcheck>::SharedPtr unloadingcheck_service;
//patrol On / Off Service//
tetra_msgs::srv::Patrol patrol_cmd;
rclcpp::Service<tetra_msgs::srv::Patrol>::SharedPtr patrol_service;
tetra_msgs::srv::PatrolConveyor patrol_conveyor_cmd;
rclcpp::Service<tetra_msgs::srv::PatrolConveyor>::SharedPtr patrol_conveyor_service;
//conveyor movement Service Client
rclcpp::Client<tetra_msgs::srv::ConveyorAutoMovement>::SharedPtr Conveyor_cmd_client;
auto conveyor_srv = std::make_shared<tetra_msgs::srv::ConveyorAutoMovement::Request>();
//delete data all Service//
tetra_msgs::srv::Deletedataall deletedataall_cmd;
rclcpp::Service<tetra_msgs::srv::Deletedataall>::SharedPtr deletedataall_service;
//Set EKF & IMU Reset Service//
tetra_msgs::srv::Setekf set_ekf_cmd;
rclcpp::Service<tetra_msgs::srv::Setekf>::SharedPtr set_ekf_service;

//**Command srv _ Service Client************************/
//Usb_cam Service Client//
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr usb_cam_On_client;
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr usb_cam_Off_client;
auto m_request = std::make_shared<std_srvs::srv::Empty::Request>();
//Charging Port on/off Service Client//
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr charging_port_On_client;
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr charging_port_Off_client;
auto m_request2 = std::make_shared<std_srvs::srv::Empty::Request>();
//request_nomotion_update 
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr request_nomotion_update_client;
auto m_request3 = std::make_shared<std_srvs::srv::Empty::Request>();
//Robot Pose Reset msgs
std_msgs::msg::Int32 tetra_PoseRest;
//LED Control Service Client// (tetraDS_interface package)
rclcpp::Client<tetra_msgs::srv::Ledcontrol>::SharedPtr led_cmd_client;
auto led_srv = std::make_shared<tetra_msgs::srv::Ledcontrol::Request>();
rclcpp::Client<tetra_msgs::srv::Ledtogglecontrol>::SharedPtr ledtoggle_cmd_client;
auto ledtoggle_srv = std::make_shared<tetra_msgs::srv::Ledtogglecontrol::Request>();
rclcpp::Client<tetra_msgs::srv::Toggleon>::SharedPtr turnon_cmd_client;
auto turnon_srv = std::make_shared<tetra_msgs::srv::Toggleon::Request>();
//goto goal id//
//TODO/////////
actionlib_msgs::GoalID goto_goal_id;
//////////////
//Clear costmap Service Client//
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmap_client;
//robot_localization Service Client//
rclcpp::Client<tetra_msgs::srv::SetPose>::SharedPtr SetPose_cmd_client;
auto setpose_srv = std::make_shared<tetra_msgs::srv::SetPose::Request>();
geometry_msgs::msg::PoseWithCovarianceStamped set_pose;

//sonar sensor
rclcpp::Client<tetra_msgs::srv::PowerSonarCmd>::SharedPtr power_sonar_cmd_client;
auto Power_sonar_srv = std::make_shared<tetra_msgs::srv::PowerSonarCmd::Request>();

//Bumper_data to Pointcloud2_data//
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
sensor_msgs::msg::PointCloud2 pointcloud_;
float P_INF_X = 0.1;  // somewhere out of reach from the robot (positive x)
float P_INF_Y = + P_INF_X*cos(0.34906585);  // somewhere out of reach from the robot (positive y)
float N_INF_Y = - P_INF_X*cos(0.34906585);  // somewhere out of reach from the robot (negative y)
float ZERO = 0.0;
float pc_radius_ = 0.05;
float pc_height_ = 0.0;
float p_side_x_ = 0.05;
float p_side_y_ = + p_side_x_*cos(0.34906585);
float n_side_y_ = - p_side_x_*cos(0.34906585);

//conveyor_test...
int  m_iLoading_ID = 0;
int  m_iUnloading_ID = 0;
string m_strLoading_loacation_name = "LOADING";
string m_strUnloading_loacation_name = "UNLOADING";

//Ignition point landmark add..
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_pub;
visualization_msgs::msg::Marker node;

//IMU Service Client//
rclcpp::Client<tetra_msgs::srv::AllDataReset>::SharedPtr all_data_reset_cmd_client;
auto all_data_reset_srv = std::make_shared<tetra_msgs::srv::AllDataReset::Request>();
rclcpp::Client<tetra_msgs::srv::EulerAngleInit>::SharedPtr euler_angle_init_cmd_client;
auto euler_angle_init_srv = std::make_shared<tetra_msgs::srv::EulerAngleInit::Request>();
rclcpp::Client<tetra_msgs::srv::EulerAngleReset>::SharedPtr euler_angle_reset_cmd_client;
auto euler_angle_reset_srv = std::make_shared<tetra_msgs::srv::EulerAngleReset::Request>();
rclcpp::Client<tetra_msgs::srv::PoseVelocityReset>::SharedPtr pose_velocity_reset_cmd_client;
auto pose_velocity_reset_srv = std::make_shared<tetra_msgs::srv::PoseVelocityReset::Request>();
rclcpp::Client<tetra_msgs::srv::RebootSensor>::SharedPtr reboot_sensor_cmd_client;
auto reboot_sensor_srv = std::make_shared<tetra_msgs::srv::RebootSensor::Request>();


//ros Node
std::shared_ptr<rclcpp::Node> nodes;

//parameter setting
auto double_param = rcl_interfaces::msg::Parameter();
auto double_request = std::make_shared<rcl_interaces::srv::SetParametersAtomically::Request>();
rclcpp::Client<rcl_interaces::srv::SetParametersAtomically>::SharedPtr param_double_client;
string double_service_name = "";
auto int_param = rcl_interfaces::msg::Parameter();
auto int_request = std::make_shared<rcl_interaces::srv::SetParametersAtomically::Request>();
rclcpp::Client<rcl_interaces::srv::SetParametersAtomically>::SharedPtr param_int_client;
string int_service_name = "";
auto bool_param = rcl_interfaces::msg::Parameter();
auto bool_request = std::make_shared<rcl_interaces::srv::SetParametersAtomically::Request>();
rclcpp::Client<rcl_interaces::srv::SetParametersAtomically>::SharedPtr param_bool_client1;
rclcpp::Client<rcl_interaces::srv::SetParametersAtomically>::SharedPtr param_bool_client2;
string bool_service_name1 = "";
string bool_service_name2 = "";

//************************************************************************************************************************//

void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n",s);
    exit(1); 
}

/** reduce angle to between 0 and 360 degrees. */
double reduceHeading(double a) 
{
    return remainder(a-180, 360)+180;
}

///////Logitech F710 Joypad//////////////////////////////////////
void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if(joy->buttons[6]) //LT button
	{	
        //Docking Start//
		ex_iDocking_CommandMode = 1;
        _pFlag_Value.m_bfalg_DockingExit = false;
	}
    if(joy->buttons[7]) //RT button
	{	
		//Docking Stop//
		ex_iDocking_CommandMode = 0;
        _pFlag_Value.m_bfalg_DockingExit = true;

	}
	
}

///////SICK TIM 571  Range Check//////////////////////////////////////
void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int size = msg->ranges.size();

    //Right Check//
    int R_minIndex = 10;
    int R_maxIndex = 185;
    int R_closestIndex = -1;
    double R_minVal = 0.2;

    for (int i = R_minIndex; i < R_maxIndex; i++)
    {
        if ((msg->ranges[i] <= R_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            R_minVal = msg->ranges[i];
            R_closestIndex = i;
        }
    }
    //printf("R_closestIndex: %d || check: %f \n" , R_closestIndex, msg->ranges[R_closestIndex]);
    if(R_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_Right = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_Right = false;

    /**************************************************************************************************************************/
    //Center Check//
    int C_minIndex = 320; //180;
    int C_maxIndex = 420; //550;
    int C_closestIndex = -1;
    double C_minVal = 0.8; //0.3

    for (int i = C_minIndex; i < C_maxIndex; i++)
    {
        if ((msg->ranges[i] <= C_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            C_minVal = msg->ranges[i];
            C_closestIndex = i;
        }
    }
    //printf("C_closestIndex: %d || check: %f \n" , C_closestIndex, msg->ranges[C_closestIndex]);
    if(C_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_Center = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_Center = false;

    /**************************************************************************************************************************/
    //Left Check//
    int L_minIndex = 555;
    int L_maxIndex = 730;
    int L_closestIndex = -1;
    double L_minVal = 0.2;

    for (int i = L_minIndex; i < L_maxIndex; i++)
    {
        if ((msg->ranges[i] <= L_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            L_minVal = msg->ranges[i];
            L_closestIndex = i;
        }
    }
    //printf("L_closestIndex: %d || check: %f \n" , L_closestIndex, msg->ranges[L_closestIndex]);
    if(L_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_Left = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_Left = false;

}

void PCL1_Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int size = msg->ranges.size();
    //printf("PCL1_size: %d \n",size); //848
    int PCL1_minIndex = 0;
    int PCL1_maxIndex = 848;
    int PCL1_closestIndex = -1;
    double PCL1_minVal = 0.6;

    for (int i = PCL1_minIndex; i < PCL1_maxIndex; i++)
    {
        if ((msg->ranges[i] <= PCL1_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            PCL1_minVal = msg->ranges[i];
            PCL1_closestIndex = i;
        }
    }
    //printf("PCL1_closestIndex: %d || check: %f \n" , PCL1_closestIndex, msg->ranges[PCL1_closestIndex]);
    if(PCL1_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_PCL1 = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_PCL1 = false;

}

void PCL2_Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int size = msg->ranges.size();
    //printf("PCL2_size: %d \n",size);
    int PCL2_minIndex = 0;
    int PCL2_maxIndex = 848;
    int PCL2_closestIndex = -1;
    double PCL2_minVal = 0.6;

    for (int i = PCL2_minIndex; i < PCL2_maxIndex; i++)
    {
        if ((msg->ranges[i] <= PCL2_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            PCL2_minVal = msg->ranges[i];
            PCL2_closestIndex = i;
        }
    }
    //printf("PCL1_closestIndex: %d || check: %f \n" , PCL1_closestIndex, msg->ranges[PCL1_closestIndex]);
    if(PCL2_closestIndex > 0)
        _pFlag_Value.m_bFlag_Obstacle_PCL2 = true;
    else
        _pFlag_Value.m_bFlag_Obstacle_PCL2 = false;
}

double Quaternion2Yaw(double Quaternion_W, double Quaternion_X, double Quaternion_Y, double Quaternion_Z)
{
    double m_dYaw_deg = 0.0;
    double m_dRoll = 0.0;
    double m_dPitch = 0.0;
    double m_dYaw = 0.0;

    //< Declaration of quaternion
    tf2::Quaternion q;
    q.setW(Quaternion_W);
    q.setX(Quaternion_X);
    q.setY(Quaternion_Y);
    q.setZ(Quaternion_Z);
    //< quaternion -> rotation Matrix
    tf2::Matrix3x3 m(q);
    //< rotation Matrix - > quaternion
    m.getRotation(q);
    //< rotation Matrix -> rpy
    m.getRPY(m_dRoll, m_dPitch, m_dYaw);
    m_dYaw_deg = m_dYaw * (180.0/M_PI);
    if(m_dYaw_deg < 0)
    {
        m_dYaw_deg = -1.0 * m_dYaw_deg;
    }
    
    return m_dYaw_deg;
}

double Quaternion2Yaw_rad(double Quaternion_W, double Quaternion_X, double Quaternion_Y, double Quaternion_Z)
{
    double m_dYaw_rad = 0.0;
    double m_dRoll = 0.0;
    double m_dPitch = 0.0;
    double m_dYaw = 0.0;

    //< Declaration of quaternion
    tf2::Quaternion q;
    q.setW(Quaternion_W);
    q.setX(Quaternion_X);
    q.setY(Quaternion_Y);
    q.setZ(Quaternion_Z);

    //< quaternion -> rotation Matrix
    tf2::Matrix3x3 m(q);
    //< rotation Matrix - > quaternion
    m.getRotation(q);
    //< rotation Matrix -> rpy
    m.getRPY(m_dRoll, m_dPitch, m_dYaw);
    m_dYaw_rad = m_dYaw; 
    
    //m_dYaw_rad = reduceHeading(m_dYaw);
    return m_dYaw_rad;
}


string GetWIFI_IPAddress()
{
    string str_ip;
    int fd;
    struct ifreq ifr;
    char myEth0_addr[20] = {0,};
 
    fd = socket(AF_INET, SOCK_DGRAM, 0);
     
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, "enp2s0", IFNAMSIZ -1); //Ehernet device name Check
    
    ioctl(fd, SIOCGIFADDR, &ifr);
    close(fd);
     
    sprintf(myEth0_addr, "%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
    
    str_ip = myEth0_addr;
    return str_ip;
}

int old_slopTime = 0;
bool AccelerationControl(int slopTime)
{
    if(slopTime < 0) return false;

    if(old_slopTime != slopTime)
    {
        accel_vel.data = slopTime;
        Accel_pub.publish(accel_vel);
        printf(" AccelerationControl %d : %d \n", old_slopTime, slopTime);
    }

    old_slopTime = slopTime;
    return true;
}

void Docking_EXIT()
{
    cmd.linear.x =  0.0; 
    cmd.angular.z = 0.0;
    ex_iDocking_CommandMode = 0;
    printf("[EXIT]: Docking Exit !! \n");
}

bool DockingStop_Command(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    //Docking_EXIT();
	return true;
}

bool SlopTime_Command(const std::shared_ptr<tetra_msgs::srv::Accelerationslop::Request> req, 
				      std::shared_ptr<tetra_msgs::srv::Accelerationslop::Response> res)
{
	bool bResult = false;

	AccelerationControl(req->slop_time);
	/*
	int32 slop_time
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Servo_Command(const std::shared_ptr<tetra_msgs::srv::Servo::Request> req, 
				   std::shared_ptr<tetra_msgs::srv::Servo::Response> res)
{
	bool bResult = false;

	servo_request.data = req->data; 
	servo_pub->publish(servo_request);
	bResult = true;
	/*
	int32 data
	---
	bool command_Result
	*/
	res->command_result = bResult;
	return true;
}

string DRD_old_strname = "";
double DRD_old_dValue = 0.0;
void Dynamic_reconfigure_Teb_Set_DoubleParam(string strname, double dValue)
{
    if((DRD_old_strname != strname) || (DRD_old_dValue != dValue))
    {
        m_flag_Dynamic_reconfigure_call = true;

        double_param.name = strname;
        double_param.value.type = 3;
        double_param.value.double_value = dValue;
        double_request->paameters.push_back(double_param);

        while(!param_double_client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(nodes->get_logger(), "service " << double_service_name << " not available, waiting again...");
        }
        auto result = param_double_client->async_send_request(double_request);

        m_flag_Dynamic_reconfigure_call = false;

        double_request->paameters.clear();
    }
    DRD_old_strname = strname;
    DRD_old_dValue = dValue;
}

string DRI_old_strname = "";
int DRI_old_iValue = 0;
void Dynamic_reconfigure_Costmap_Set_IntParam(string strname, int iValue)
{
    if((DRI_old_strname != strname) || (DRI_old_iValue != iValue))
    {
        int_param.name = strname;
        int_param.value.type = 2;
        int_param.value.integer_value = iValue;
        int_request->paameters.push_back(int_param);

        while(!param_int_client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(nodes->get_logger(), "service " << int_service_name << " not available, waiting again...");
        }
        auto result = param_int_client->async_send_request(int_request);

        int_request->paameters.clear();
    }
    DRI_old_strname = strname;
    DRI_old_iValue = iValue;
}

string DRB_old_strname = "";
bool DRB_old_bValue = false;
void Dynamic_reconfigure_Costmap_Set_BoolParam(string strname, bool bValue)
{
    if((DRB_old_strname != strname) || (DRB_old_bValue != bValue))
    {
        bool_param.name = strname;
        bool_param.value.type = 1;
        bool_param.value.bool_value = bValue;
        bool_request->paameters.push_back(bool_param);

        while(!param_bool_client1->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(nodes->get_logger(), "service " << bool_service_name1 << " not available, waiting again...");
        }
        auto result = param_bool_client1->async_send_request(bool_request);

        while(!param_bool_client2->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(nodes->get_logger(), "service " << bool_service_name2 << " not available, waiting again...");
        }
        auto result = param_bool_client2->async_send_request(bool_request);

        bool_request->paameters.clear();
    }
    DRB_old_strname = strname;
    DRB_old_bValue = bValue;
}

void Sonar_On(int iOn)
{
    Power_sonar_srv->start = iOn;
    while(!power_sonar_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service power sonar cmd not available, waiting again...");
    }
    auto result = power_sonar_cmd_client->async_send_request(Power_sonar_srv);
}

void LED_Turn_On(int id)
{
    turnon_srv->id = id;
    while(!turnon_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service led turn on cmd not available, waiting again...");
    }
    auto result = turnon_cmd_client->async_send_request(turnon_srv);
}

void LED_Control(int id, int led_brightness)
{
    led_srv->id = id;
    led_srv->led_brightness = led_brightness;
    while(!led_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service led cmd not available, waiting again...");
    }
    auto result = led_cmd_client->async_send_request(led_srv);
}

void LED_Toggle_Control(int de_index, int light_acc, int High_brightness, int light_dec, int Low_brightness)
{
    ledtoggle_srv->de_index = de_index;
    ledtoggle_srv->light_accel = light_acc;
    ledtoggle_srv->led_high_brightness = High_brightness;
    ledtoggle_srv->light_decel = light_dec;
    ledtoggle_srv->led_low_brightness = Low_brightness;
    while(!ledtoggle_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service led toggle cmd not available, waiting again...");
    }
    auto result = ledtoggle_cmd_client->async_send_request(ledtoggle_srv);
}

bool Setspeed_Command(const std::shared_ptr<tetra_msgs::srv::Setmaxspeed::Request> req, 
				      std::shared_ptr<tetra_msgs::srv::Setmaxspeed::Response> res)
{
	bool bResult = false;

	Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", (double)req->speed);
	_pDynamic_param.MAX_Linear_velocity = (double)req->speed;

	/*
	float32 speed
	---
	float32 set_vel
	bool command_Result
	*/
	res->set_vel = req->speed;
	bResult = true;
	res->command_result = bResult;
	return true;
}

void cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    _pDynamic_param.m_linear_vel  = msg->linear.x;
    _pDynamic_param.m_angular_vel = msg->angular.z;
}

void Particle_Callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    m_iParticleCloud_size = msg->poses.size();
    if(m_iParticleCloud_size > 501 && _pDynamic_param.m_linear_vel == 0.0 && _pDynamic_param.m_angular_vel == 0.0 && _pFlag_Value.m_bFlag_Initialpose)
    {
        if(_pFlag_Value.m_bFlag_nomotion)
        {
	        m_bFlag_nomotion_call = true;
            while(!request_nomotion_update_client->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO_STREAM(nodes->get_logger(), "service nomotion update not available, waiting again...");
            }
            auto result = request_nomotion_update_client->async_send_request(m_request3);
        }   
    }
    else
    {
        m_bFlag_nomotion_call = false;
	    _pFlag_Value.m_bFlag_Initialpose = false;
    }
    
}

void TebMarkers_Callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    float m_point_vector = 0.0;
    if(msg->ns == "ViaPoints")
    {
        m_iViaPoint_Index = msg->points.size();
        //printf("!!!m_iIndex: %d \n", m_iIndex);
        if(m_iViaPoint_Index <= 1)
        {
            if(!_pFlag_Value.m_bTebMarker_reconfigure_flag)
            {
                if(_pDynamic_param.m_linear_vel >= 0.3)
                {
                    if(!m_flag_Dynamic_TebMarkers_major_update)
                    {
                        Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", 0.3);
                        m_flag_Dynamic_TebMarkers_major_update = true;
                        _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
                    }
                }
            }
        }
        else
        {
            _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            m_flag_Dynamic_TebMarkers_major_update = false;
        }
        _pFlag_Value.m_bflagGo = true;
    }
    else
    {
        _pFlag_Value.m_bflagGo = false;
    }
}

void Teblocalplan_Callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    double m_dDelta_Value = 0.0;

    int m_iPoseIndex = msg->poses.size();
    for(int i=0; i<m_iPoseIndex; i++)
    {
        m_dTeb_Pose_head_Angle[i] = Quaternion2Yaw(msg->poses[i].orientation.w, 
                                                    msg->poses[i].orientation.x, 
                                                    msg->poses[i].orientation.y, 
                                                    msg->poses[i].orientation.z);
    }

    m_dDelta_Value = m_dTeb_Pose_head_Angle[1] - m_dTeb_Pose_head_Angle[0];
    if(m_dDelta_Value < 0)
    {
        m_dDelta_Value = -1.0 * m_dDelta_Value;
    }
    if(_pFlag_Value.m_bCorneringFlag)
    {
        if(m_dDelta_Value >= 3.5)
        {
            if(!m_flag_Dynamic_Teblocalplan_major_update)
            {
                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_theta", 0.5); //0.35
                m_flag_Dynamic_Teblocalplan_major_update = true;
                m_flag_Dynamic_Teblocalplan_minor_update = false;
                _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
            }
            else
            {
                _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            }
            
        }
        else
        {
            if(!m_flag_Dynamic_Teblocalplan_minor_update)
            {
                Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_theta", 0.15);
                m_flag_Dynamic_Teblocalplan_minor_update = true;
                m_flag_Dynamic_Teblocalplan_major_update = false;
                _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
            }
            else
            {
                _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
            }
        }
    }
}

//TODO
void setGoal(move_base_msgs::MoveBaseActionGoal& goal)
{
    ros::Time now = ros::Time::now();

    goal.header.frame_id="map";
    goal.header.stamp=now;
    goal.goal_id.stamp = now;
    goal.goal.target_pose.header.stamp = now;
    goal.goal.target_pose.header.frame_id = "map";
    goal.goal.target_pose.pose.position.x = _pGoal_pose.goal_positionX;
    goal.goal.target_pose.pose.position.y = _pGoal_pose.goal_positionY;
    goal.goal.target_pose.pose.position.z = _pGoal_pose.goal_positionZ;
    goal.goal.target_pose.pose.orientation.x = _pGoal_pose.goal_quarterX;
    goal.goal.target_pose.pose.orientation.y = _pGoal_pose.goal_quarterY;
    goal.goal.target_pose.pose.orientation.z = _pGoal_pose.goal_quarterZ;
    goal.goal.target_pose.pose.orientation.w = _pGoal_pose.goal_quarterW;

    service_pub.publish(goal);
    printf("setGoal call: %.5f, %.5f !!\n", _pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY);
    _pFlag_Value.m_bFlag_pub = true;
}

bool SaveLocation(string str_location, 
                  float m_fposeAMCLx, float m_fposeAMCLy,
                  float m_fposeAMCLqx, float m_fposeAMCLqy, float m_fposeAMCLqz, float m_fposeAMCLqw)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    {
        RCLCPP_INFO(nodes->get_logger(), "file is null");
        bResult = false;
    }
    else
    {
        fprintf(fp, "0,%lf,%lf,%lf,%lf,%lf,%lf \n",
                m_fposeAMCLx, m_fposeAMCLy, m_fposeAMCLqx, m_fposeAMCLqy, m_fposeAMCLqz, m_fposeAMCLqw);
        fclose(fp);
        bResult = true;
    }

    return bResult;
}

bool OpenLocationFile(string str_location)
{
    bool bResult = false;
    string m_strFilePathName;

    if(str_location == "HOME")
    {
        _pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
        _pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
        _pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
        _pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
        _pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
        _pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
        _pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;
   
        bResult = true; 
    }
    else
    {
        m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";  
        fp = fopen(m_strFilePathName.c_str(), "r");  

        if(fp != NULL) //File Open Check
        {
            while(!feof(fp))
            {
                if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
                {
                    char* ptr = strtok(Textbuffer, ",");
                    int icnt = 0;

                    while(ptr != NULL)
                    {   
                        ptr = strtok(NULL, ",");
                        switch(icnt)
                        {
                            case 0:
                                if(ptr != NULL)
                                {
                                    _pGoal_pose.goal_positionX = atof(ptr);
                                }
                                break;
                            case 1:
                                _pGoal_pose.goal_positionY = atof(ptr);
                                break;
                            case 2:
                                _pGoal_pose.goal_quarterX = atof(ptr);
                                break;
                            case 3:
                                _pGoal_pose.goal_quarterY = atof(ptr);
                                break;
                            case 4:
                                _pGoal_pose.goal_quarterZ = atof(ptr);  
                                break;
                            case 5:
                                _pGoal_pose.goal_quarterW = atof(ptr);
                                break;
                        }
                        icnt++;
                    }
                    bResult = true; 
                }
                else
                {
                    bResult = false; 
                }
            }                
            fclose(fp);
        }
        else
        {
            RCLCPP_INFO(nodes->get_logger(), "File Open Fail: %s", str_location.c_str());
            bResult = false;
        }
    }

    return bResult;
}

bool GetLocation_Command(const std::shared_ptr<tetra_msgs::srv::Getlocation::Request> req, 
					     std::shared_ptr<tetra_msgs::srv::Getlocation::Response> res)
{
	bool bResult = false;

	//Get POSE
	// res->poseAMCLx = _pAMCL_pose.poseAMCLx;
	// res->poseAMCLy = _pAMCL_pose.poseAMCLy;
	// res->poseAMCLqx = _pAMCL_pose.poseAMCLqx;
	// res->poseAMCLqy = _pAMCL_pose.poseAMCLqy;
	// res->poseAMCLqz = _pAMCL_pose.poseAMCLqz;
	// res->poseAMCLqw = _pAMCL_pose.poseAMCLqw;

	res->pose_amcl_x = _pTF_pose.poseTFx;
	res->pose_amcl_y = _pTF_pose.poseTFy;
	res->pose_amcl_qx = _pTF_pose.poseTFqx;
	res->pose_amcl_qy = _pTF_pose.poseTFqy;
	res->pose_amcl_qz = _pTF_pose.poseTFqz;
	res->pose_amcl_qw = _pTF_pose.poseTFqw;
	/*
	---
	bool   command_Result
	float  poseAMCLx
	float  poseAMCLy
	float  poseAMCLqx
	float  poseAMCLqy
	float  poseAMCLqz
	float  poseAMCLqw
	*/
	res->command_result = bResult;
	return true;
}

bool Depart_Station2Move()
{
    bool bResult = false;
    
    if(_pAR_tag_pose.m_transform_pose_x <= 0.7) //700mm depart move
    {
        if(_pFlag_Value.m_bFlag_Obstacle_Center)
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            bResult = false;
        }
        else
        {
            cmd->linear.x =  0.05; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            bResult = false;
        }
            
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);

        //add goto cmd call//
        //TODO
        setGoal(goal);

        bResult = true;
        ex_iDocking_CommandMode = 0;
    }
    
    return bResult;
}

bool Goto_Command(const std::shared_ptr<tetra_msgs::srv::Gotolocation::Request> req, 
				  std::shared_ptr<tetra_msgs::srv::Gotolocation::Response> res)
{
	bool bResult = false;
	m_flag_setgoal = true;

	//costmap clear call//
    while(!clear_costmap_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
    }
    auto result = clear_costmap_client->async_send_request(m_request);
    if(rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        m_flag_clesr_costmap_call = true;
    }
    m_flag_clesr_costmap_call = false;

	LED_Toggle_Control(1,3,100,3,1);
	if(_pFlag_Value.m_bflag_patrol)
		LED_Turn_On(100); //blue led
	else
		LED_Turn_On(63); //White led

	bResult = OpenLocationFile(req->location);
	printf("Goto bResult: %d \n", bResult);
	res->goal_position_x = _pGoal_pose.goal_positionX;
	res->goal_position_y = _pGoal_pose.goal_positionY;
	res->goal_quarter_x  = _pGoal_pose.goal_quarterX;
	res->goal_quarter_y  = _pGoal_pose.goal_quarterY;
	res->goal_quarter_z  = _pGoal_pose.goal_quarterZ;
	res->goal_quarter_w  = _pGoal_pose.goal_quarterW;
    //TODO
	goto_goal_id.id = req->location;

    RCLCPP_INFO(nodes->get_logger(), "goto_id.id: %s", goto_goal_id.id.c_str());

    if(_pRobot_Status.m_iCallback_Charging_status <= 1 && (_pAR_tag_pose.m_iAR_tag_id == -1 || _pAR_tag_pose.m_transform_pose_x <= 0.5)) //Nomal
    {
        RCLCPP_INFO(nodes->get_logger(), "Goto Nomal Loop !");
        LED_Toggle_Control(1,3,100,3,1);
        if(_pFlag_Value.m_bflag_patrol)
            LED_Turn_On(100); //blue led
        else
            LED_Turn_On(63); //White led

        setGoal(goal);
        bResult = true;
        
    }
    
    else //Docking...
    {
        ex_iDocking_CommandMode = 10; //Depart Move
        bResult = true;
    }

    if(req->Location == "HOME") //HOME Point Check
    {
        _pFlag_Value.m_bflag_ComebackHome = true;
    }

	/*
	string Location
	int32 mark_id
	int32 movement
	---
	float64 goal_positionX
	float64 goal_positionY
	float64 goal_quarterX
	float64 goal_quarterY
	float64 goal_quarterZ
	float64 goal_quarterW
	bool command_Result
	*/
	res->command_result = bResult;

	//reset flag...
	_pFlag_Value.m_bFlag_Disable_bumper = false;
	_pFlag_Value.m_bFlag_Initialpose = false;
	
	m_flag_setgoal = false;

	return true;
}

bool Goto_Command2(const std::shared_ptr<tetra_msgs::srv::Gotolocation2::Request> req, 
				   std::shared_ptr<tetra_msgs::srv::Gotolocation2::Response> res)
{
	bool bResult = false;
	
	m_flag_setgoal = true;

	//costmap clear call//
	while(!clear_costmap_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
    }
    auto result = clear_costmap_client->async_send_request(m_request);
    if(rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        m_flag_clesr_costmap_call = true;
    }
	m_flag_clesr_costmap_call = false;

	_pGoal_pose.goal_positionX = req->goal_position_x;
	_pGoal_pose.goal_positionY = req->goal_position_y;
	_pGoal_pose.goal_quarterX = req->goal_quarter_x;
	_pGoal_pose.goal_quarterY = req->goal_quarter_y;
	_pGoal_pose.goal_quarterZ = req->goal_quarter_z;
	_pGoal_pose.goal_quarterW = req->goal_quarter_w;
	//goto_goal_id.id = "1";

	LED_Toggle_Control(1, 3,100,3,1);
	LED_Turn_On(63);

	if(_pRobot_Status.m_iCallback_Charging_status <= 1 && (_pAR_tag_pose.m_iAR_tag_id == -1 || _pAR_tag_pose.m_transform_pose_x <= 0.5)) //Nomal
	{
        //TODO
		setGoal(goal);
		bResult = true;
	}
	else //Docking...
	{
		ex_iDocking_CommandMode = 10; //Depart Move
		bResult = true;//false;
	}
	/*
	float goal_positionX
	float goal_positionY
	float goal_quarterX
	float goal_quarterY
	float goal_quarterZ
	float goal_quarterW
	---
	bool command_Result
	*/
	res->command_result = bResult;
    //reset flag...
    _pFlag_Value.m_bFlag_Disable_bumper = false;
	_pFlag_Value.m_bFlag_Initialpose = false;
	
	m_flag_setgoal = false;
	
	return true;
}

bool GotoCancel_Command(const std::shared_ptr<tetra_msgs::srv::Gotocancel::Request> req, 
				        std::shared_ptr<tetra_msgs::srv::Gotocancel::Response> res)
{
	bool bResult = false;
	
	m_flag_setgoal = true;
    //TODO
	goto_goal_id.id = "";
    RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
	GotoCancel_pub.publish(goto_goal_id);
	/*
	string Location_id
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	ex_iDocking_CommandMode = 0;
	
	m_flag_setgoal = false;
    _pFlag_Value.m_bFlag_pub = false;
	return true;
}

bool SetLocation_Command(const std::shared_ptr<tetra_msgs::srv::Setlocation::Request> req, 
					     std::shared_ptr<tetra_msgs::srv::Setlocation::Response> res)
{
	bool bResult = false;

    if(req->location == "HOME")
    {
        res->command_result = false;
        printf("[ERROR]: HOME location cannot be saved! \n");
    }
    else
    {
        /*
        res->command_result = SaveLocation(req->Location, 
                                        _pAMCL_pose.poseAMCLx, _pAMCL_pose.poseAMCLy,
                                        _pAMCL_pose.poseAMCLqx, _pAMCL_pose.poseAMCLqy, _pAMCL_pose.poseAMCLqz, _pAMCL_pose.poseAMCLqw);
        
        res->goal_positionX = _pAMCL_pose.poseAMCLx;
        res->goal_positionY = _pAMCL_pose.poseAMCLy;
        res->goal_quarterX  = _pAMCL_pose.poseAMCLqx;
        res->goal_quarterY  = _pAMCL_pose.poseAMCLqy;
        res->goal_quarterZ  = _pAMCL_pose.poseAMCLqz;
        res->goal_quarterW  = _pAMCL_pose.poseAMCLqw;
        */

        res->command_result = SaveLocation(req->location, 
                                _pTF_pose.poseTFx,_pTF_pose.poseTFy,
                                _pTF_pose.poseTFqx,_pTF_pose.poseTFqy,_pTF_pose.poseTFqz,_pTF_pose.poseTFqw);
        
        res->goal_position_x = _pTF_pose.poseTFx;
        res->goal_position_y = _pTF_pose.poseTFy;
        res->goal_quarter_x  = _pTF_pose.poseTFqx;
        res->goal_quarter_y  = _pTF_pose.poseTFqy;
        res->goal_quarter_z  = _pTF_pose.poseTFqz;
        res->goal_quarter_w  = _pTF_pose.poseTFqw;
    }

	/*
	string Location
	---
	float goal_positionX
	float goal_positionY
	float goal_quarterX
	float goal_quarterY
	float goal_quarterZ
	float goal_quarterW
	bool command_Result
	*/
	res->command_result = bResult;

	return true;
}

void CMD_SaveMap(string strMapname)
{
    //call rosrun command//
    string str_command = "gnome-terminal -- ros2 run nav2_map_server map_saver_cli --occ 55 --free 45 -f ";
    string str_command2 = str_command + strMapname;
    std::vector<char> writable1(str_command2.begin(), str_command2.end());
    writable1.push_back('\0');
    char* ptr1 = &writable1[0];

    int iResult = std::system(ptr1);
}


bool SetSavemap_Command(const std::shared_ptr<tetra_msgs::srv::Setsavemap::Request> req, 
					    std::shared_ptr<tetra_msgs::srv::Setsavemap::Response> res)
{
	bool bResult = false;

    RCLCPP_INFO(nodes->get_logger(), "Save Map Call _ %s", req->map_name.c_str())

	//call rosrun command//
	string str_command = "gnome-terminal -- /home/tetra/mapsave.sh ";
	string str_command2 = str_command + req->map_name.c_str();

	std::vector<char> writable1(str_command2.begin(), str_command2.end());
	writable1.push_back('\0');
	char* ptr1 = &writable1[0];

	int iResult = std::system(ptr1);
	/*
	string map_name
	---
	bool command_Result
	*/
	res->command_result = bResult;
	//bResult = true;
	return true;
}

bool GetInformation_Command(const std::shared_ptr<tetra_msgs::srv::Getinformation::Request> req, 
					        std::shared_ptr<tetra_msgs::srv::Getinformation::Response> res)
{
	bool bResult = false;
	res->command_result = false;

	//Get Data
	res->battery = _pRobot_Status.m_iCallback_Battery;
	res->error_code = _pRobot_Status.m_iCallback_ErrorCode;
	res->emg = _pRobot_Status.m_iCallback_EMG;
	res->bumper = _pRobot_Status.m_iCallback_Bumper;
	res->charging = _pRobot_Status.m_iCallback_Charging_status;
	res->running_mode = ex_ilaunchMode; //0:nomal, 1:mapping, 2:navigation
	/*
	---
	bool command_Result
	int32 battery
	int32 Error_Code
	bool EMG
	bool bumper
	bool charging
	*/

	res->command_result = bResult;
	return true;
}

bool Docking_Command(const std::shared_ptr<tetra_msgs::srv::Dockingcontrol::Request> req, 
					 std::shared_ptr<tetra_msgs::srv::Dockingcontrol::Response> res)
{
	bool bResult = false;
	_pAR_tag_pose.m_iSelect_AR_tag_id = req->id;
	//_pRobot_Status.HOME_ID = _pAR_tag_pose.m_iSelect_AR_tag_id = req->id;
	ex_iDocking_CommandMode = req->mode;

	/*
	int32 id
	int32 mode
	---
	bool command_Result
	*/

	res->command_result = bResult;
	return true;
}

void poseAMCLCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msgAMCL)
{
    _pAMCL_pose.poseAMCLx = msgAMCL->pose.pose.position.x;
    _pAMCL_pose.poseAMCLy = msgAMCL->pose.pose.position.y;
    _pAMCL_pose.poseAMCLz = msgAMCL->pose.pose.position.z;
    _pAMCL_pose.poseAMCLqx = msgAMCL->pose.pose.orientation.x;
    _pAMCL_pose.poseAMCLqy = msgAMCL->pose.pose.orientation.y;
    _pAMCL_pose.poseAMCLqz = msgAMCL->pose.pose.orientation.z;
    _pAMCL_pose.poseAMCLqw = msgAMCL->pose.pose.orientation.w;

}


bool LocationList_Command(const std::shared_ptr<tetra_msgs::srv::Getlocationlist::Request> req, 
					      std::shared_ptr<tetra_msgs::srv::Getlocationlist::Response> res)
{
    bool bResult = false;

    int m_icnt =0;
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/DATA/"); //file path
    if (dir != NULL) 
    {
        res->command_result = true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            m_icnt++;
            char *listname = strchr(ent->d_name, '.');
            if(listname != NULL){
                *listname = 0;
            }
            //printf ("%s\n", ent->d_name);
            res->location_name.push_back(ent->d_name);

        }

        //Total List number
        res->list_num = m_icnt;
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        res->command_result = false;
    }

    /*
    ---
    int32 list_num
    string[] location_name
    bool command_Result
    */

    res->command_result = bResult;
    return true;
}

bool LandmarkList_Command(const std::shared_ptr<tetra_msgs::srv::Getlandmarklist::Request> req, 
					      std::shared_ptr<tetra_msgs::srv::Getlandmarklist::Response> res)
{
    bool bResult = false;

    int m_icnt =0;
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/LANDMARK/"); //file path
    if (dir != NULL) 
    {
        res->command_result = true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            m_icnt++;
            char *listname = strchr(ent->d_name, '.');
            if(listname != NULL){
                *listname = 0;
            }
            //printf ("%s\n", ent->d_name);
            res->landmark_name.push_back(ent->d_name);

        }

        //Total List number
        res->list_num = m_icnt;
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        res->command_result = false;
    }

    res->command_result = bResult;
    return true;
}

bool MapList_Command(const std::shared_ptr<tetra_msgs::srv::Getmaplist::Request> req, 
					 std::shared_ptr<tetra_msgs::srv::Getmaplist::Response> res)
{
    bool bResult = false;

    //Load File List
    int m_icnt =0;
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/ros2_ws/src/tetra_pkg/tetra_2dnav/maps/"); //file path

    if (dir != NULL) 
    {
        res->command_result = true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            if(strstr(ent->d_name, ".yaml") == 0)
            {
                m_icnt++;
                char *listname = strchr(ent->d_name, '.');
                if(listname != NULL){
                    *listname = 0;
                }
                //printf("%s\n", ent->d_name);
                res->map_name.push_back(ent->d_name);
            }

        }

        //Total List number
        res->list_num = m_icnt;
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        res->command_result = false;
    }

    /*
    string IP_address
    ---
    int32 list_num
    string[] map_name
    bool command_Result
    */

    res->command_result = bResult;
    return true;
}

bool DeleteLocation_Command(const std::shared_ptr<tetra_msgs::srv::Deletelocation::Request> req, 
				           std::shared_ptr<tetra_msgs::srv::Deletelocation::Response> res)
{
    bool bResult = false;

    string m_strLocationName;
    m_strLocationName = "/home/tetra/DATA/" + req->location + ".txt";  
    int iResult = remove(m_strLocationName.c_str());
    if(iResult == 0)
    {
        res->command_result = true;
    }
    else
    {
        res->command_result = false;
    }
    
    /*
    string Location
    ---
    bool command_Result
    */

    res->command_result = bResult;
    return true;
}

bool DeleteLandmark_Command(const std::shared_ptr<tetra_msgs::srv::Deletelandmark::Request> req, 
				           std::shared_ptr<tetra_msgs::srv::Deletelandmark::Response> res)
{
    bool bResult = false;

    string m_strLandmarkName;
    m_strLandmarkName = "/home/tetra/LANDMARK/" + req->landmark + ".txt";  
    
    int iResult = remove(m_strLandmarkName.c_str());
    if(iResult == 0)
    {
        res->command_result = true;
    }
    else
    {
        res->command_result = false;
    }
    

    res->command_result = bResult;
    return true;
}

bool DeleteMap_Command(const std::shared_ptr<tetra_msgs::srv::Deletemap::Request> req, 
				      std::shared_ptr<tetra_msgs::srv::Deletemap::Response> res)
{
    bool bResult = false;

    string m_strMapName_yaml;
    string m_strMapName_pgm;
    m_strMapName_yaml = "/home/tetra/ros2_ws/src/tetra_pkg/tetra_2dnav/maps/" + req->map_name + ".yaml";  
    m_strMapName_pgm  = "/home/tetra/ros2_ws/src/tetra_pkg/tetra_2dnav/maps/" + req->map_name + ".pgm";

    //yaml file erase
    int iResult1 = remove(m_strMapName_yaml.c_str());
    if(iResult1 == 0)
    {
        res->command_result = true;
    }
    else
    {
        res->command_result = false;
    }
    //pgm file erase
    int iResult2 = remove(m_strMapName_pgm.c_str());
    if(iResult2 == 0)
    {
        res->command_result = true;
    }
    else
    {
        res->command_result = false;
    }
    
    /*
    string map_name
    ---
    bool command_Result
    */
    res->command_result = true;
    return true;
}

bool Virtual_Obstacle_Command(const std::shared_ptr<tetra_msgs::srv::VirtualObstacle::Request> req, 
				              std::shared_ptr<tetra_msgs::srv::VirtualObstacle::Response> res)
{
	bool bResult = false;
	int m_iInt_count = 0;
	int m_iNext_count = 0;

	// msg clear
	virtual_obstacle.list.clear();
	//Global_costmap Loop//
	virtual_obstacle.list.resize(req->list_count.size());
	for(int i=0; i<req->list_count.size(); i++)
	{
		virtual_obstacle.list[i].form.clear();
		virtual_obstacle.list[i].form.resize(req->list_count[i]);
		m_iInt_count = req->list_count[i];
		for(int j=0; j<req->list_count[i]; j++)
		{
		    virtual_obstacle.list[i].form[j].x = floor(req->form_x[m_iNext_count + j] * 1000.f + 0.5) / 1000.f;
		    virtual_obstacle.list[i].form[j].y = floor(req->form_y[m_iNext_count + j] * 1000.f + 0.5) / 1000.f;
		    virtual_obstacle.list[i].form[j].z = floor(req->form_z[m_iNext_count + j] * 1000.f + 0.5) / 1000.f;
		}
		m_iNext_count += m_iInt_count;

	}

	if(m_bFlag_nomotion_call || !_pFlag_Value.m_bFlag_nomotion || m_flag_Dynamic_reconfigure_call || m_flag_setgoal || _pFlag_Value.m_bTebMarker_reconfigure_flag)
	{
		bResult = false;
		res->command_result = bResult;
		//printf("!!!! m_flag_Dynamic_reconfigure_call Timing !!!! \n");
		return true;
	}
	virtual_obstacle_pub->publish(virtual_obstacle);
 
	/*
	int32[]  list_count
	float64[] form_x
	float64[] form_y
	float64[] form_z
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Patrol_Command(const std::shared_ptr<tetra_msgs::srv::Patrol::Request> req, 
				    std::shared_ptr<tetra_msgs::srv::Patrol::Response> res)
{
	bool bResult = false;

	_pFlag_Value.m_bflag_patrol = req->on;
	m_patrol_location_cnt = req->list_count;
	printf("# m_bflag_patrol: %d  \n", _pFlag_Value.m_bflag_patrol);
	printf("# m_patrol_location_cnt: %d  \n", m_patrol_location_cnt);

	for(int i=0; i<m_patrol_location_cnt; i++)
	{
		arr_patrol_location[i] = req->location_name[i];
        RCLCPP_INFO(nodes->get_logger(), "# arr_patrol_location[%d]:%s ", i, arr_patrol_location[i].c_str());
	}
   
	/*
	bool on
	int32  list_count
	string[] location_name
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Patrol_Conveyor_Command(const std::shared_ptr<tetra_msgs::srv::PatrolConveyor::Request> req, 
				             std::shared_ptr<tetra_msgs::srv::PatrolConveyor::Response> res)
{
	bool bResult = false;

	_pFlag_Value.m_bflag_patrol2 = req->on;
	m_iLoading_ID = req->loading_id;
	m_iUnloading_ID = req->unloading_id;
	m_strLoading_loacation_name = req->loading_location_name;
	m_strUnloading_loacation_name = req->unloading_location_name;

	/*
	bool on
	int32  loading_id
	int32  unloading_id
	string loading_location_name
	string unloading_location_name
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	return true;
}

void Reset_EKF_SetPose()
{
	//robot_localization::SetPose ekf_reset;
	setpose_srv->pose.header.frame_id = tf_prefix_ + "/odom";
	setpose_srv->pose.header.stamp = nodes->get_clock()->now(); //ros::Time::now();

	setpose_srv->pose.pose.pose.position.x = 0.0; //_pTF_pose.poseTFx;
	setpose_srv->pose.pose.pose.position.y = 0.0; //_pTF_pose.poseTFy;
	setpose_srv->pose.pose.pose.position.z = 0.0; //_pTF_pose.poseTFz;

	setpose_srv->pose.pose.pose.orientation.x = 0.0;
	setpose_srv->pose.pose.pose.orientation.y = 0.0;
	setpose_srv->pose.pose.pose.orientation.z = 0.0; //_pTF_pose.poseTFqz;
	setpose_srv->pose.pose.pose.orientation.w = 1.0; //_pTF_pose.poseTFqw;

	setpose_srv->pose.pose.covariance[0] = 0.25;
	setpose_srv->pose.pose.covariance[6 * 1 + 1] = 0.25;
	setpose_srv->pose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    while(!SetPose_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service set pose cmd not available, waiting again...");
    }
    auto result = SetPose_cmd_client->async_send_request(setpose_srv); //Set_pose call//
	printf("##Set_Pose(EKF)! \n");

	initPose_.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
	initPose_.header.frame_id = "map";
	//position
	initPose_.pose.pose.position.x = 0.0;
	initPose_.pose.pose.position.y = 0.0;
	initPose_.pose.pose.position.z = 0.0;
	//orientation
	initPose_.pose.pose.orientation.x = 0.0;
	initPose_.pose.pose.orientation.y = 0.0;
	initPose_.pose.pose.orientation.z = 0.0;
	initPose_.pose.pose.orientation.w = 1.0;

	initPose_.pose.covariance[0] = 0.25;
	initPose_.pose.covariance[6 * 1 + 1] = 0.25;
	initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

	initialpose_pub->publish(initPose_);
	printf("##Set_initPose(2D Estimate)! \n");

	usleep(500000);

	_pFlag_Value.m_bFlag_nomotion = true;
}

bool Marker_Reset_Robot_Pose()
{
    _pFlag_Value.m_bFlag_nomotion = false;
    
    bool bResult = false;
    string m_strFilePathName;
    string landmark_name = "marker_" + std::to_string(_pAR_tag_pose.m_iAR_tag_id);
    if (_pAR_tag_pose.m_iAR_tag_id == -1) 
    {
        bResult = false;
        return -1;
    }
    else 
    {
        printf("#Docking Reset Marker_ID: %d \n", _pAR_tag_pose.m_iAR_tag_id);
        bResult = true;
    }

   
    m_strFilePathName = "/home/tetra/LANDMARK/" + landmark_name + ".txt";
    fp = fopen(m_strFilePathName.c_str(), "r");

    if(_pAR_tag_pose.m_iAR_tag_id > 0 && fp == NULL) 
    {
        _pFlag_Value.m_bFlag_nomotion = true;
        return false;
    }

    if (fp != NULL) //File Open Check
    {
        while (!feof(fp))
        {
            if (fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
            {
                char* ptr = strtok(Textbuffer, ",");
                int icnt = 0;

                while (ptr != NULL)
                {
                    ptr = strtok(NULL, ",");
                    switch (icnt)
                    {
                    case 6:
                        _pLandMarkPose.init_position_x = atof(ptr);
                        break;
                    case 7:
                        _pLandMarkPose.init_position_y = atof(ptr);
                        break;
                    case 8:
                        _pLandMarkPose.init_position_z = atof(ptr);
                        break;
                    case 9:
                        _pLandMarkPose.init_orientation_z = atof(ptr);
                        break;
                    case 10:
                        _pLandMarkPose.init_orientation_w = atof(ptr);
                        break;
                    }

                    icnt++;
                }
            }
        }
        fclose(fp);
	    initPose_.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
	    initPose_.header.frame_id = "map";
	    //position
	    initPose_.pose.pose.position.x = _pLandMarkPose.init_position_x;
	    initPose_.pose.pose.position.y = _pLandMarkPose.init_position_y;
	    initPose_.pose.pose.position.z = _pLandMarkPose.init_position_z;
	    //orientation
	    initPose_.pose.pose.orientation.x = 0.0;
	    initPose_.pose.pose.orientation.y = 0.0;
	    initPose_.pose.pose.orientation.z = _pLandMarkPose.init_orientation_z;
	    initPose_.pose.pose.orientation.w = _pLandMarkPose.init_orientation_w;

	    initPose_.pose.covariance[0] = 0.25;
	    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
	    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

        //publish msg
        initialpose_pub->publish(initPose_);

        printf("$$$$ init_position_x: %f , init_position_y: %f \n", _pLandMarkPose.init_position_x, _pLandMarkPose.init_position_y);
    }
    else
    {
        bResult = false;
    }

    _pFlag_Value.m_bFlag_nomotion = true;

    return bResult;
}


void Reset_Robot_Pose()
{
    if(_pRobot_Status.HOME_ID == _pAR_tag_pose.m_iAR_tag_id) // Same as Home ID... Loop 
    {
        _pFlag_Value.m_bFlag_nomotion = false;

        //IMU reset//
        while(!euler_angle_reset_cmd_client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(nodes->get_logger(), "service euler angle reset cmd not available, waiting again...");
        }
        auto result = euler_angle_reset_cmd_client->async_send_request(euler_angle_reset_srv);
	    printf("## IMU Reset ! \n");
        //tetra odometry Reset//
        tetra_PoseRest.data = m_iReset_flag;
        PoseReset_pub->publish(tetra_PoseRest);
        usleep(100000);

        Reset_EKF_SetPose();
    }

    Marker_Reset_Robot_Pose();

    //costmap clear call//
    //clear_costmap_client.call(m_request);

    virtual_obstacle2.list.clear();
    virtual_obstacle2_pub->publish(virtual_obstacle2);

    _pFlag_Value.m_bFlag_Initialpose = true;
    
}

bool SetInitPose_Command(const std::shared_ptr<tetra_msgs::srv::Setinitpose::Request> req, 
					     std::shared_ptr<tetra_msgs::srv::Setinitpose::Response> res)
{
    _pFlag_Value.m_bFlag_nomotion = false;

    bool bResult = false;

    string landmark_name = "marker_" + std::to_string(_pAR_tag_pose.m_iAR_tag_id);
    if(_pAR_tag_pose.m_iAR_tag_id == -1)
    {
        bResult = false; 
    }
    else
    {
        res->m_iar_tag_id = _pAR_tag_pose.m_iAR_tag_id;
        bResult = true; 
    }

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/LANDMARK/" + landmark_name + ".txt";  
    fp = fopen(m_strFilePathName.c_str(), "r");  

    if(fp != NULL) //File Open Check
    {
        while(!feof(fp))
        {
            if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
            {
                char* ptr = strtok(Textbuffer, ",");
                int icnt = 0;

                while(ptr != NULL)
                {   
                    ptr = strtok(NULL, ",");
                    switch(icnt)
                    {
                        case 6:
                            _pLandMarkPose.init_position_x = atof(ptr);
                            res->init_position_x = _pLandMarkPose.init_position_x;
                            break;
                        case 7:
                            _pLandMarkPose.init_position_y = atof(ptr);
                            res->init_position_y = _pLandMarkPose.init_position_y;
                            break;
                        case 8:
                            _pLandMarkPose.init_position_z = atof(ptr);
                            res->init_position_z = _pLandMarkPose.init_position_z;
                            break;
                        case 9:
                            _pLandMarkPose.init_orientation_z = atof(ptr);
                            res->init_orientation_z = _pLandMarkPose.init_orientation_z;
                            break;
                        case 10:
                            _pLandMarkPose.init_orientation_w = atof(ptr);
                            res->init_orientation_w = _pLandMarkPose.init_orientation_w;
                            break;
                    }
        
                    icnt++;
                }
            }
        }                
        fclose(fp);
    }

    initPose_.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = _pLandMarkPose.init_position_x;
    initPose_.pose.pose.position.y = _pLandMarkPose.init_position_y;
    initPose_.pose.pose.position.z = _pLandMarkPose.init_position_z;
    //orientation
    initPose_.pose.pose.orientation.x = 0.0;
    initPose_.pose.pose.orientation.y = 0.0;
    initPose_.pose.pose.orientation.z = _pLandMarkPose.init_orientation_z;
    initPose_.pose.pose.orientation.w = _pLandMarkPose.init_orientation_w;
    
    initPose_.pose.covariance[0] = 0.25;
    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    //publish msg
    initialpose_pub->publish(initPose_);
    //costmap clear call//
    while(!clear_costmap_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
    }
    auto result = clear_costmap_client->async_send_request(m_request);

    res->command_result = bResult;
    _pFlag_Value.m_bFlag_nomotion = true;

    return true;
}

bool Set2D_Pose_Estimate_Command(const std::shared_ptr<tetra_msgs::srv::PoseEstimate::Request> req, 
					             std::shared_ptr<tetra_msgs::srv::PoseEstimate::Response> res)
{
    _pFlag_Value.m_bFlag_nomotion = false;

    bool bResult = false;
    initPose_.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
    initPose_.header.frame_id = "map";
    //position
    initPose_.pose.pose.position.x = req->estimate_position_x;
    initPose_.pose.pose.position.y = req->estimate_position_y;
    initPose_.pose.pose.position.z = req->estimate_position_z;
    //orientation
    initPose_.pose.pose.orientation.x = req->estimate_orientation_x;
    initPose_.pose.pose.orientation.y = req->estimate_orientation_y;
    initPose_.pose.pose.orientation.z = req->estimate_orientation_z;
    initPose_.pose.pose.orientation.w = req->estimate_orientation_w;
    
    initPose_.pose.covariance[0] = 0.25;
    initPose_.pose.covariance[6 * 1 + 1] = 0.25;
    initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    //publish msg
    initialpose_pub->publish(initPose_);
    //costmap clear call//
    while(!clear_costmap_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
    }
    auto result = clear_costmap_client->async_send_request(m_request);
    res->command_result = bResult;
    _pFlag_Value.m_bFlag_nomotion = true;
/*
    float64 estimate_position_x
    float64 estimate_position_y
    float64 estimate_position_z
    float64 estimate_orientation_x
    float64 estimate_orientation_y
    float64 estimate_orientation_z
    float64 estimate_orientation_w
    ---
    bool command_Result
*/
    return true;
}

void ChargingCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot_Status.m_iCallback_Charging_status = msg->data; //docking_status
    
    if(_pRobot_Status.m_iCallback_Charging_status > 1 && _pFlag_Value.m_Onetime_reset_flag == false )
    {
        _pFlag_Value.m_Onetime_reset_flag = true;
    }
    
    if(_pRobot_Status.m_iCallback_Charging_status <= 1)
    {
        _pFlag_Value.m_Onetime_reset_flag = false;
    }
}

void BatteryCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot_Status.m_iCallback_Battery = msg->data;
}

void EMGCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot_Status.m_iCallback_EMG = msg->data;
    if(_pRobot_Status.m_iCallback_EMG != 0)
    {
        LED_Toggle_Control(1, 10,100,10,1);
        LED_Turn_On(18);
        printf("[EMG] Push EMG button!! _ RED LED On \n");
        _pFlag_Value.m_emgpush_flag = false;
    }
    else
    {
        if(!_pFlag_Value.m_emgpush_flag)
        {
            LED_Toggle_Control(1, 3,100,3,1);
            LED_Turn_On(63);
            _pFlag_Value.m_emgpush_flag = true;
        }
    }
}

void BumperCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot_Status.m_iCallback_Bumper = msg->data;

    if(_pRobot_Status.m_iCallback_Bumper != 0)
    {
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
        memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
        pointcloud_.header.stamp = nodes->get_clock()->now();
        pointcloud_pub_->publish(pointcloud_);

        if(!_pFlag_Value.m_bFlag_Disable_bumper)
        {
            LED_Toggle_Control(1, 10,100,10,1);
            LED_Turn_On(18);
            printf("[Bumper] Push Bumper!! _ RED LED On \n");
       
            if(_pFlag_Value.m_bflagGo)
            {
                goto_goal_id.id = "";
                //TODO
                RCLCPP_INFO(nodes->get_logger(), "[Bumper On]Goto Cancel call");
                GotoCancel_pub->publish(goto_goal_id);
                _pFlag_Value.m_bflagGo = false;
                _pFlag_Value.m_bflagGo2 = true;
                if(_pFlag_Value.BUMPER_BT)
                    ex_iDocking_CommandMode = 100;
            }
        }
        _pFlag_Value.m_bumperhit_flag = true;
    }
    else
    { 
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
        memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
        memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
        memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
        pointcloud_.header.stamp = nodes->get_clock()->now();
        pointcloud_pub_->publish(pointcloud_);

        if(_pFlag_Value.m_bumperhit_flag)
        {
            LED_Toggle_Control(1,3,100,3,1);
            LED_Turn_On(63); //White led
            _pFlag_Value.m_bumperhit_flag = false;
        }

    }

}

void Ultrasonic_DL_Callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    m_Ultrasonic_DL_Range = msg->range;
}

void Ultrasonic_DR_Callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    m_Ultrasonic_DR_Range = msg->range;
}

void Ultrasonic_RL_Callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    m_Ultrasonic_RL_Range = msg->range;
}

void Ultrasonic_RR_Callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    m_Ultrasonic_RR_Range = msg->range;
}

//Conveyor function
void LoadcellCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    _pRobot_Status.m_dLoadcell_weight = msg->data;
}

void SensorCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot_Status.m_iConveyor_Sensor_info = msg->data;
}

/////////LanMark file Write & Read Fuction///////////
bool SaveLandMark(LANDMARK_POSE p)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/tetra/LANDMARK/" + p.ns + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    { 
        RCLCPP_INFO(nodes->get_logger(), "file is null");
        bResult = false;
    }
    else
    {
        fprintf(fp, "0,%s,%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",
                p.header_frame_id.c_str(), p.ns.c_str(), p.mark_id,
                p.pose_position_x, p.pose_position_y, p.pose_position_z, 
                p.pose_orientation_x, p.pose_orientation_y, p.pose_orientation_z, p.pose_orientation_w);
        fclose(fp);

        bResult = true;
    }

    return bResult;
}

//AR_tagCallback
void AR_tagCallback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr req) 
{
    if (!req->markers.empty()) 
    {
        //add...find ID array Index Loop...
        for (int i = 0; i < req->markers.size(); i++)
        {
            if (_pAR_tag_pose.m_iSelect_AR_tag_id == req->markers[i].id)
            {
                _pAR_tag_pose.m_iAR_tag_id_Index = i;
            }
        }

        //AR_Tag data update...
        _pAR_tag_pose.m_iAR_tag_id = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].id;
        _pAR_tag_pose.m_fAR_tag_pose_x = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.x;
        _pAR_tag_pose.m_fAR_tag_pose_y = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.y;
        _pAR_tag_pose.m_fAR_tag_pose_z = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.position.z;
        _pAR_tag_pose.m_fAR_tag_orientation_x = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.x;
        _pAR_tag_pose.m_fAR_tag_orientation_y = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.y;
        _pAR_tag_pose.m_fAR_tag_orientation_z = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.z;
        _pAR_tag_pose.m_fAR_tag_orientation_w = req->markers[_pAR_tag_pose.m_iAR_tag_id_Index].pose.pose.orientation.w;
        
        //< Declaration of quaternion
        tf2::Quaternion q;
        q.setW(_pAR_tag_pose.m_fAR_tag_orientation_w);
        q.setX(_pAR_tag_pose.m_fAR_tag_orientation_x);
        q.setY(_pAR_tag_pose.m_fAR_tag_orientation_y);
        q.setZ(_pAR_tag_pose.m_fAR_tag_orientation_z);
        //< quaternion -> rotation Matrix
        tf2::Matrix3x3 m(q);
        //< rotation Matrix - > quaternion
        m.getRotation(q);
        //< rotation Matrix -> rpy
        m.getRPY(_pAR_tag_pose.m_fAR_tag_roll, _pAR_tag_pose.m_fAR_tag_pitch, _pAR_tag_pose.m_fAR_tag_yaw);
        _pAR_tag_pose.m_fPositioning_Angle = _pAR_tag_pose.m_fAR_tag_pitch * (180.0/M_PI);

        //Transform Axis
        _pAR_tag_pose.m_transform_pose_x = _pAR_tag_pose.m_fAR_tag_pose_z;
        _pAR_tag_pose.m_transform_pose_y = _pAR_tag_pose.m_fAR_tag_pose_x;

    }
    else
    {
        _pAR_tag_pose.m_iAR_tag_id = -1;
    }  
}

//TODO
void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msgResult)
{
   uint8_t PENDING    = 0;   // The goal has yet to be processed by the action server
   uint8_t ACTIVE     = 1;   // The goal is currently being processed by the action server
   uint8_t PREEMPTED  = 2;   // The goal received a cancel request after it started executing
                             //   and has since completed its execution (Terminal State)
   uint8_t SUCCEEDED  = 3;   // The goal was achieved successfully by the action server (Terminal State)
   uint8_t ABORTED    = 4;   // The goal was aborted during execution by the action server due
                             //    to some failure (Terminal State)
   uint8_t REJECTED   = 5;   // The goal was rejected by the action server without being processed,
                             //    because the goal was unattainable or invalid (Terminal State)
   uint8_t PREEMPTING = 6;   // The goal received a cancel request after it started executing
                             //    and has not yet completed execution
   uint8_t RECALLING  = 7;   // The goal received a cancel request before it started executing,
                             //    but the action server has not yet confirmed that the goal is canceled
   uint8_t RECALLED   = 8;   // The goal received a cancel request before it started executing
                             //    and was successfully cancelled (Terminal State)
   uint8_t LOST       = 9;   // An action client can determine that a goal is LOST. This should not be
                             //    sent over the wire by an action server
   time_t curr_time;
   struct tm *curr_tm;
   curr_time = time(NULL);
   curr_tm = localtime(&curr_time);

  if(msgResult->status.status == SUCCEEDED)
  { 
    ROS_INFO("[SUCCEEDED]resultCallback: %d ",msgResult->status.status);
    _pFlag_Value.m_bflag_NextStep = true;
    _pRobot_Status.m_iMovebase_Result = 3;
    m_iRetry_cnt = 0;
    //LED_Turn_On(4);
    LED_Control(1, 100);
    usleep(100000); 
    LED_Control(2, 100);
    //costmap clear call//
    //clear_costmap_client.call(m_request);

    if(_pFlag_Value.m_bflag_ComebackHome) //Home Postion -> docking mode start
    {
        _pAR_tag_pose.m_iSelect_AR_tag_id = _pRobot_Status.HOME_ID; //0;
        ex_iDocking_CommandMode = 1;
        _pFlag_Value.m_bflag_ComebackHome = false;
    } 

    if(_pFlag_Value.m_bflag_Conveyor_docking) //Conveyor Postion -> docking mode start
    {
        _pAR_tag_pose.m_iSelect_AR_tag_id = _pRobot_Status.CONVEYOR_ID;
        ex_iDocking_CommandMode = 11;
        _pFlag_Value.m_bflag_Conveyor_docking = false;
    }

    m_flag_PREEMPTED = false;
    _pFlag_Value.m_bFlag_pub = false;

  }
  else if( msgResult->status.status == ABORTED)
  {
    LED_Toggle_Control(1, 10,100,10,1);
    LED_Turn_On(18);
    printf("[ERROR]resultCallback _ RED LED On \n");
    _pFlag_Value.m_bflag_NextStep = false;
    ROS_INFO("[ERROR]resultCallback: %d ",msgResult->status.status);
	  
    m_flag_setgoal = true;

    goto_goal_id.id = "";
    ROS_INFO("Goto Cancel call");
    GotoCancel_pub.publish(goto_goal_id);
    _pFlag_Value.m_bFlag_pub = false;

    if(m_iRetry_cnt >= MAX_RETRY_CNT)
    {
        m_iRetry_cnt = 0;
        ROS_INFO("[RETRY Behavior]: FAIL !");
    }
    else
    {
	//costmap clear call//
    	clear_costmap_client.call(m_request);
	    
        LED_Toggle_Control(1, 3,100,3,1);
        if(_pFlag_Value.m_bflag_Conveyor_docking)
            LED_Turn_On(45); //sky_blue
        else
            LED_Turn_On(63);
        
        ROS_INFO("[RETRY Behavior]: goto_ %s", goal.goal_id.id.c_str());
        setGoal(goal);
        m_iRetry_cnt++;
    }

    m_flag_setgoal = false;
    m_flag_PREEMPTED = false;

  }

  else if(msgResult->status.status == PREEMPTED) //bumper On Check...
  {
    //goto_goal_id.id = "";
    //ROS_INFO("Goto Cancel call");
    //GotoCancel_pub.publish(goto_goal_id);
    m_flag_PREEMPTED = true;
  }

  else
  {
    _pFlag_Value.m_bflag_NextStep = false;
    ROS_INFO("resultCallback: %d ",msgResult->status.status);
    _pRobot_Status.m_iMovebase_Result = msgResult->status.status;
    //costmap clear call//
    //clear_costmap_client.call(m_request);
  }
}

// add nav2 Die Check
bool checkNode(std::string& node_name)
{
    std::vector<std::string> node_list;

    node_list = nodes->get_node_names();

    for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
    {
        if (node_list[node_list_idx] == node_name)
        return true;
    }

    RCLCPP_ERROR(nodes->get_logger(), "nav2 error");
    cmd->linear.x = 0.0;
    cmd->angular.z = 0.0;
    cmdpub_->publish(cmd);
    _pFlag_Value.m_bFlag_pub = true;
    return false;
}

//TODO
void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msgStatus)
{
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9
    if(!msgStatus->status_list.empty())
    {
        actionlib_msgs::GoalStatus goalStatus = msgStatus->status_list[0];
        status_id = goalStatus.status;
        //ROS_INFO("[move_base]StatusCallback: %d ",status_id);
    }
    else
    {
        if(_pFlag_Value.m_bFlag_pub)
        {
            ROS_INFO("move_base die Catch !! ");
            //Stop
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            //Retry setGoal..
            if(_pGoal_pose.goal_positionX != 0.0 && _pGoal_pose.goal_positionY != 0.0)
            {
                setGoal(goal);
            }
            printf("setGoal call: %.5f, %.5f !!\n", _pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY);
            _pFlag_Value.m_bFlag_pub = false;
        }
    }
}

constexpr unsigned int HashCode(const char* str)
{
    return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * HashCode(str + 1) : 8603;
}

bool terminal_call()
{
    bool m_Result = false;

    string str_command = "gnome-terminal";
    std::vector<char> writable(str_command.begin(), str_command.end());
    writable.push_back('\0');
    char* ptr = &writable[0];
    int iResult = std::system(ptr);

    m_Result = true;
    return m_Result;
}

bool rosrun_mapping()
{
    bool m_Result = false;

    string str_command = "gnome-terminal -- /home/tetra/mapping.sh ";
    std::vector<char> writable2(str_command.begin(), str_command.end());
    writable2.push_back('\0');
    char* ptr2 = &writable2[0];
    int iResult = std::system(ptr2);

    ex_ilaunchMode = 1;

    m_Result = true;
    return m_Result;
}

bool rosnodekill_all()
{
    bool m_Result = false;

    std::vector<std::string> node_list;

    node_list = nodes->get_node_names();

    for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
    {
        if (node_list[node_list_idx] == "amcl"
         || node_list[node_list_idx] == "nav2_controller"
         || node_list[node_list_idx] == "tetra_landmark"
         || node_list[node_list_idx] == "nav2_map_server" 
         || node_list[node_list_idx] == "world_linker" 
         || node_list[node_list_idx] == "cartographer_ros" 
         || node_list[node_list_idx] == "rviz2" 
         || node_list[node_list_idx] == "ar_track_alvar2")
         {
            string str_command = "ros2 lifecycle set " + node_list[node_list_idx] + " shutdown";
            std::vector<char> writable3(str_command.begin(), str_command.end());
            writable3.push_back('\0');
            char* ptr3 = &writable3[0];
            int iResult = std::system(ptr3);
         }
    }

    ex_ilaunchMode = 0;

    m_Result = true;
    return m_Result;
}

bool rosrun_navigation(string strMapname)
{
    bool m_Result = false;

    string str_command = "gnome-terminal -- /home/tetra/navigation.sh ";
    string str_command2 = str_command + strMapname.c_str();
    std::vector<char> writable4(str_command2.begin(), str_command2.end());
    writable4.push_back('\0');
    char* ptr4 = &writable4[0];
    int iResult = std::system(ptr4);

    ex_ilaunchMode = 2;

    m_Result = true;
    return m_Result;
}

bool Approach_Station2Move()
{
    bool bResult = false;

    if(_pRobot_Status.m_iCallback_Charging_status <= 1)
    {
        if(m_iDocking_timeout_cnt > 3000)
        {
            cmd->linear.x = 0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            m_iDocking_timeout_cnt = 0;
            ex_iDocking_CommandMode = 9;

        }
        else
        {
            cmd->linear.x = -0.01; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            m_iDocking_timeout_cnt++;
            
        }
    }
    else
    {
        cmd->linear.x = 0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        printf("Approach Loop STOP !! \n");
        ex_iDocking_CommandMode = 6;
    }
    
    bResult = true;
    return bResult;
}

bool Approach_Station2Move2()
{
    bool bResult = false;

    if(_pRobot_Status.m_iCallback_Charging_status <= 1)
    {
        if(m_iDocking_timeout_cnt > 3000)
        {
            cmd->linear.x = 0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            m_iDocking_timeout_cnt = 0;
            ex_iDocking_CommandMode = 9;

        }
        else
        {
            cmd->linear.x = -0.01; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            m_iDocking_timeout_cnt++;
            
        }

    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        printf("Approach Loop STOP !! \n");
        ex_iDocking_CommandMode = 16;
    }
    
    bResult = true;
    return bResult;
}

double Rotation_Movement()
{
    double iResult = 0.1;

    if(m_Ultrasonic_DR_Range < 0.3 )
    {
        m_iRotation_Mode = 1;
        printf(" CCW Rotation--- \n");
    }
    else if(m_Ultrasonic_DL_Range < 0.3)
    {
        m_iRotation_Mode = 2;
        printf(" CW Rotation+++ \n");
    }

    switch(m_iRotation_Mode)
    {
        case 0:
            iResult = 0.1;
            break;
        case 1:
            iResult = -0.1;
            break;
        case 2:
            iResult = 0.1;
            break;

    }

    return iResult;
}

bool BumperCollision_Behavior()
{
    bool bResult = false;

    if(_pRobot_Status.m_iBumperCollisionBehavor_cnt > 500)
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        
        if(_pFlag_Value.m_bflagGo2)
        {
            LED_Toggle_Control(1, 3,100,3,1);
            LED_Turn_On(63);
            //TODO
            ROS_INFO("[RETRY Behavior2]: goto_ %s", goal.goal_id.id.c_str());
            setGoal(goal);
            _pFlag_Value.m_bflagGo2 = false;
        }
        
        _pRobot_Status.m_iBumperCollisionBehavor_cnt = 0;
        ex_iDocking_CommandMode = 0;

        bResult = true;
    }
    else
    {
        if((m_Ultrasonic_RL_Range <= 0.2) || (m_Ultrasonic_RR_Range <= 0.2))
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
        }
        else
        {
            cmd->linear.x =  -0.02; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            _pRobot_Status.m_iBumperCollisionBehavor_cnt++;
        } 
    }

    return bResult;
}

/****************************************************************/
//Charging Station Docking Function//
bool ChargingStation_tracking(bool bOn, int marker_id)
{
    bool bResult = false;

    //Todo.....
    if(bOn)
    {
        float m_fdistance = 0.0;
        if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
        {
            m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
            //printf("master_distance: %.5f \n", m_fdistance);
            if(m_fdistance > 0.41 && m_fdistance < 1.5)
            {
                cmd->linear.x = -1.0 * (m_fdistance /1.2) * 0.15; 
                //printf("linear velocity: %.2f \n", cmd->linear.x);
                if(cmd->linear.x > 1.0)
                {
                    //Linear Over speed exit loop...
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Linear Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
                //printf("angular velocity: %.2f \n", cmd-angular.z);
                if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
                {
                    //Angular Over speed exit loop......
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Angular Over speed]: follower is closing \n");
                    return false;
                }
                
                cmdpub_->publish(cmd);
            }
            else
            {
                cmd->linear.x =  0.0; 
                cmdpub_->publish(cmd);
                printf("Tracking STOP !! \n");
                ex_iDocking_CommandMode = 3;
                m_iNoMarker_cnt = 0;
            }
        }
        else
        {
            printf("No Marker, Rotation Movement !! \n");
            cmd->angular.z = Rotation_Movement(); //0.1;
            cmdpub_->publish(cmd);

            if(m_iNoMarker_cnt > 4000) //retry timeout!!
            {
                m_iNoMarker_cnt = 0;
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_->publish(cmd);
                printf("DockingStation scan Fail !! \n");
                ex_iDocking_CommandMode = 9;
            }
            else
            {
                m_iNoMarker_cnt++;
            }
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        printf("Docking Loop STOP!_not find Marker!! \n");
        ex_iDocking_CommandMode = 0;
    }
    
    bResult = true;
    return bResult;
}

bool ChargingStation_Yaw_tracking()
{ 
    bool bResult = false;
    
    int m_iback_cnt = 0;
    float m_fdistance = 0.0;
    m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);

    if(_pAR_tag_pose.m_target_yaw <= 0.0174533 && _pAR_tag_pose.m_target_yaw >= -0.0174533) //+- 1.0deg
    {
        ex_iDocking_CommandMode = 4;
        bResult = true;
        return bResult;
    }

    if(_pAR_tag_pose.m_target_yaw > 0)
    {
        printf("++dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_->publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_->publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }


        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_Center)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_->publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = m_fdistance * 0.2;
                cmdpub_->publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }

        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_->publish(cmd);
        printf("++dir STOP \n");
        m_iRotation_Mode = 2;
        ex_iDocking_CommandMode = 2;
    }
    else
    {
        printf("--dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_->publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_->publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_Center)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_->publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = m_fdistance * 0.2;
                cmdpub_->publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }

        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_->publish(cmd);
        printf("--dir STOP \n");
        m_iRotation_Mode = 1;
        ex_iDocking_CommandMode = 2;
    }
    

    bResult = true;
    return bResult;
}

bool ChargingStation_tracking2(int marker_id)
{
    bool bResult = false;
    
    float m_fdistance = 0.0;
    if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
    {
        m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
        //printf("master_distance: %.5f \n", m_fdistance);
        if(_pRobot_Status.m_iCallback_Charging_status < 2)
        {
            cmd->linear.x = -1.0 * (m_fdistance /1.2) * 0.1; //max speed 0.1m/s
            //printf("linear velocity: %.2f \n", cmd->linear.x);
            if(cmd->linear.x > 1.0)
            {
                //Linear Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Linear Over speed]: follower is closing \n");
                return false;
            }
            
            cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
            //printf("angular velocity: %.2f \n", cmd->angular.z);
            if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
            {
                //Angular Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Angular Over speed]: follower is closing \n");
                return false;
            }
            
            cmdpub_->publish(cmd);
        }
        else
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            printf("Tracking STOP & Docking Finish !! \n");
            ex_iDocking_CommandMode = 6; //5;
            m_iNoMarker_cnt = 0;
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        printf("No Marker 2! \n");
        if(m_iNoMarker_cnt >= 10)
        {
            m_iNoMarker_cnt = 0;
            ex_iDocking_CommandMode = 6;
            printf("No Marker 2_Timeout! \n");
        }
        else
        {
            m_iNoMarker_cnt++;
        }
    }

    bResult = true;
    return bResult;
}
/****************************************************************/
//Conveyor Station Docking Function//
bool ConveyorStation_tracking(bool bOn, int marker_id)
{
    bool bResult = false;

    if(bOn)
    {
        printf("Conveyor_ID: %d  \n", marker_id);
        float m_fdistance = 0.0;
        if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
        {
            m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
            //printf("master_distance: %.5f \n", m_fdistance);
            if(m_fdistance > 0.41 && m_fdistance < 1.5)
            {
                cmd->linear.x = -1.0 * (m_fdistance /1.2) * 0.15; 
                //printf("linear velocity: %.2f \n", cmd->linear.x);
                if(cmd->linear.x > 1.0)
                {
                    //Linear Over speed exit loop...
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Linear Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
                //printf("angular velocity: %.2f \n", cmd->angular.z);
                if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
                {
                    //Angular Over speed exit loop......
                    cmd->linear.x =  0.0; 
                    cmd->angular.z = 0.0;
                    printf("[Angular Over speed]: follower is closing \n");
                    return false;
                }
                
                cmdpub_->publish(cmd);
            }
            else
            {
                cmd->linear.x =  0.0; 
                //cmd->angular.z = 0.0;
                cmdpub_->publish(cmd);
                printf("Conveyor Tracking STOP !! \n");
                ex_iDocking_CommandMode = 13;
                m_iNoMarker_cnt = 0;
            }
        }
        else
        {
            printf("No Marker, Rotation Movement !! \n");
            cmd->angular.z = Rotation_Movement(); //0.1;
            cmdpub_->publish(cmd);
            if(m_iNoMarker_cnt > 4000) //retry timeout!!
            {
                m_iNoMarker_cnt = 0;
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                cmdpub_->publish(cmd);
                printf("ConveyorStation scan Fail !! \n");
                ex_iDocking_CommandMode = 9;

            }
            else
            {
                m_iNoMarker_cnt++;
            }

        }

    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        printf("Conveyor Docking Loop STOP!_not find Marker!! \n");
        ex_iDocking_CommandMode = 0;
    }
    bResult = true;
    return bResult;
}

bool ConveyorStation_Yaw_tracking()
{ 
    bool bResult = false;

    int m_iback_cnt = 0;
    float m_fdistance = 0.0;
    m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);

    if(_pAR_tag_pose.m_target_yaw <= 0.0174533 && _pAR_tag_pose.m_target_yaw >= -0.0174533) //+- 1.0deg
    {
        ex_iDocking_CommandMode = 14;
        bResult = true;
        return bResult;
    }

    if(_pAR_tag_pose.m_target_yaw > 0)
    {
        printf("++dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_->publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_->publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_Center)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_->publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = m_fdistance * 0.2;
                cmdpub_->publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }
        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_->publish(cmd);
        printf("++dir STOP \n");
        m_iRotation_Mode = 2;
        ex_iDocking_CommandMode = 12;
    }
    else
    {
        printf("--dir \n");
        cmd->angular.z = -1.0 * _pAR_tag_pose.m_target_yaw * 1.6;
        cmdpub_->publish(cmd);
        sleep(2);

        if(m_fdistance > 1.0 || m_fdistance < -1.0)
        {
            printf("[Error] Marker too far away !! \n");
            cmd->angular.z = 0.0;
            cmd->linear.x = 0.0;
            cmdpub_->publish(cmd);

            ex_iDocking_CommandMode = 9;
            bResult = false;
            return bResult;
        }

        while(m_iback_cnt < 30)
        {
            if(_pFlag_Value.m_bFlag_Obstacle_Center)
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = 0.0;
                cmdpub_->publish(cmd);
            }
            else
            {
                cmd->angular.z = 0.0;
                cmd->linear.x = m_fdistance * 0.2;
                cmdpub_->publish(cmd);
                m_iback_cnt++;
            }
            usleep(100000); //100ms
            
        }
        cmd->angular.z = 0.0;
        cmd->linear.x = 0.0;
        cmdpub_->publish(cmd);
        printf("--dir STOP \n");
        m_iRotation_Mode = 1;
        ex_iDocking_CommandMode = 12;
    }
    

    bResult = true;
    return bResult;
}

bool ConveyorStation_tracking2(int marker_id)
{
    bool bResult = false;
    
    float m_fdistance = 0.0;
    if(_pAR_tag_pose.m_iAR_tag_id == marker_id)
    {
        m_fdistance = sqrt(_pAR_tag_pose.m_transform_pose_x * _pAR_tag_pose.m_transform_pose_x + _pAR_tag_pose.m_transform_pose_y * _pAR_tag_pose.m_transform_pose_y);
        if(_pRobot_Status.m_iCallback_Charging_status < 2)
        {
            cmd->linear.x = -1.0 * (m_fdistance /1.2) * 0.1; //max speed 0.1m/s
            //printf("linear velocity: %.2f \n", cmd->linear.x);
            if(cmd->linear.x > 1.0)
            {
                //Linear Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Linear Over speed]: follower is closing \n");
                return false;
            }
            
            cmd->angular.z = -1.0 * atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25;
            //printf("angular velocity: %.2f \n", cmd->angular.z);
            if((cmd->angular.z > 1.0) || (cmd->angular.z < -1.0))
            {
                //Angular Over speed exit loop......
                cmd->linear.x =  0.0; 
                cmd->angular.z = 0.0;
                printf("[Angular Over speed]: follower is closing \n");
                return false;
            }
            
            cmdpub_->publish(cmd);
        }
        else
        {
            cmd->linear.x =  0.0; 
            cmd->angular.z = 0.0;
            cmdpub_->publish(cmd);
            printf("Conveyor Tracking STOP & Docking Finish !! \n");
            ex_iDocking_CommandMode = 16;
            m_iNoMarker_cnt = 0;
        }
    }
    else
    {
        cmd->linear.x =  0.0; 
        cmd->angular.z = 0.0;
        cmdpub_->publish(cmd);
        printf("No Marker 2! \n");
        if(m_iNoMarker_cnt >= 10)
        {
            m_iNoMarker_cnt = 0;
             ex_iDocking_CommandMode = 16;
            printf("No Marker 2_Timeout! \n");
        }
        else
        {
            m_iNoMarker_cnt++;
        }
    }

    bResult = true;
    return bResult;
}

bool mapping_Command(const std::shared_ptr<tetra_msgs::srv::Runmapping::Request> req, 
					 std::shared_ptr<tetra_msgs::srv::Runmapping::Response> res)
{
	bool bResult = false;
	bResult = req->flag_mapping;

	rosrun_mapping();
	/*
	bool flag_mapping
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool navigation_Command(const std::shared_ptr<tetra_msgs::srv::Runnavigation::Request> req, 
					    std::shared_ptr<tetra_msgs::srv::Runnavigation::Response> res)
{
	bool bResult = false;

	string str_mapname = req->map_name.c_str();
	rosrun_navigation(str_mapname);
	/*
	string map_name
	---
	bool command_Result
	*/

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool nodekill_Command(const std::shared_ptr<tetra_msgs::srv::Rosnodekill::Request> req, 
					  std::shared_ptr<tetra_msgs::srv::Rosnodekill::Response> res)
{
	bool bResult = false;

	//bResult = req->flag_kill;

	bResult = rosnodekill_all();
	printf("rosnodekill_all: %d \n", bResult);
	// sleep(1);

	/*
	---
	bool command_Result
	*/
	//throw std::runtime_error("####this will show up");

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Goto_Conveyor_Command(const std::shared_ptr<tetra_msgs::srv::Gotoconveyor::Request> req, 
				            std::shared_ptr<tetra_msgs::srv::Gotoconveyor::Response> res)
{
	bool bResult = false;
	//costmap clear call//
	while(!clear_costmap_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
    }
    auto result = clear_costmap_client->async_send_request(m_request);
    if(rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        m_flag_clesr_costmap_call = true;
    }
	m_flag_clesr_costmap_call = false;

	LED_Toggle_Control(1, 3,100,3,1);
	LED_Turn_On(45); //sky_blue

	bResult = OpenLocationFile(req->location);
	//printf("Goto bResult: %d \n", bResult);

    //TODO
	goto_goal_id.id = req->location;
	_pRobot_Status.CONVEYOR_ID = req->id;
	_pRobot_Status.CONVEYOR_MOVEMENT = req->movement;
	_pFlag_Value.m_bflag_Conveyor_docking = true;

	ROS_INFO("goto_conveyor_name: %s, id: %d, movement: %d", goto_goal_id.id.c_str(), _pRobot_Status.CONVEYOR_ID, _pRobot_Status.CONVEYOR_MOVEMENT);

	if(_pRobot_Status.m_iCallback_Charging_status <= 1 && (_pAR_tag_pose.m_iAR_tag_id == -1 || _pAR_tag_pose.m_transform_pose_x <= 0.5))  //Nomal
	{
		LED_Toggle_Control(1, 3,100,3,1);
		LED_Turn_On(45); //sky_blue
		setGoal(goal);
	}
	else //Docking...
	{
		ex_iDocking_CommandMode = 10; //Depart Move
	}

	/*
	string Location
	int32 id
	int32 movement
	---
	bool command_Result
	*/
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Loading_check_Command(const std::shared_ptr<tetra_msgs::srv::Loadingcheck::Request> req,
                            std::shared_ptr<tetra_msgs::srv::Loadingcheck::Response> res)
{
	bool bResult = false;
	//to do Check Loop...
	if(_pRobot_Status.m_iConveyor_Sensor_info == 0)
	{
		bResult = true;
	}
	else
	{
		bResult = false;
	}
	/*
	---
	int32 command_Result
	*/
	res->command_result = bResult;
	return true;
}

bool Unloading_check_Command(const std::shared_ptr<tetra_msgs::srv::Unloadingcheck::Request> req,
                            std::shared_ptr<tetra_msgs::srv::Unloadingcheck::Response> res)
{
	bool bResult = false;
	//to do Check Loop...
	if(_pRobot_Status.m_iConveyor_Sensor_info >= 2)
	{
		bResult = true;
	}
	else
	{
		bResult = false;
	}
	/*
	---
	int32 command_Result
	*/
	res->command_result = bResult;
	return true;
}

bool RemoveAll_map_data()
{
    bool bResult = false;

    string m_strLocationName;
    char   filename[1024];
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/ros2_ws/src/tetra_pkg/tetra_2dnav/maps/"); //file path
    if (dir != NULL) 
    {
        bResult= true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            sprintf(filename, "%s", ent->d_name);
            printf ("%s\n", filename);
            string str(filename);
            m_strLocationName = "/home/tetra/ros2_ws/src/tetra_pkg/tetra_2dnav/maps/" + str; 
            remove(m_strLocationName.c_str());
        }
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        bResult = false;
    }
}

bool RemoveAll_location_data()
{
    bool bResult = false;

    string m_strLocationName;
    char   filename[1024];
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/DATA/"); //file path
    if (dir != NULL) 
    {
        bResult= true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            sprintf(filename, "%s", ent->d_name);
            printf ("%s\n", filename);
            string str(filename);
            m_strLocationName = "/home/tetra/DATA/" + str; 
            remove(m_strLocationName.c_str());
        }
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        bResult = false;
    }
}

bool RemoveAll_landmark_data()
{
    bool bResult = false;

    string m_strLandmarkName;
    char   filename[1024];
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("/home/tetra/LANDMARK/"); //file path
    if (dir != NULL) 
    {
        bResult= true;
        //print all the files and directories within directory
        while ((ent = readdir (dir)) != NULL)
        {
            if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
            { 
                continue; 
            }
            sprintf(filename, "%s", ent->d_name);
            printf ("%s\n", filename);
            string str(filename);
            m_strLandmarkName = "/home/tetra/LANDMARK/" + str; 
            remove(m_strLandmarkName.c_str());
        }
        closedir (dir);
    } 
    else 
    {
        //could not open directory
        perror ("");
        bResult = false;
    }
}

bool DeleteData_All_Command(const std::shared_ptr<tetra_msgs::srv::Deletedataall::Request> req, 
				            std::shared_ptr<tetra_msgs::srv::Deletedataall::Response> res)
{
    bool bResult = false;
    bool bRemove_map = false;
    bool bRemove_location = false;
    bool bRemove_landmark = false;
    
    bRemove_map = RemoveAll_map_data();
    usleep(10000); //10ms
    bRemove_location = RemoveAll_location_data();
    usleep(10000); //10ms
    bRemove_landmark = RemoveAll_landmark_data();
    usleep(10000); //10ms

    if(bRemove_map && bRemove_location && bRemove_landmark)
    {
        bResult = true;
    }
    else
    {
        bResult = false;
    }
    
    /*
    ---
    bool command_Result
    */
    res->command_result = bResult;
    return true;
}

void *DockingThread_function(void *data)
{
    while(1)
    {
        switch(ex_iDocking_CommandMode)
        {
            case 0:
                break;
            /****************************************************************/
            // Station Docking Loop//
            case 1:
                LED_Toggle_Control(1, 3,100,3,100);
                LED_Turn_On(63);
                //usb_cam_On_client.call(m_request);
                sleep(2);
                ex_iDocking_CommandMode = 2;
                docking_progress.data = 1;
                docking_progress_pub->publish(docking_progress);
                break;
            case 2:
                ChargingStation_tracking(true, _pRobot_Status.HOME_ID);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 2;
                docking_progress_pub->publish(docking_progress);
                break;
            case 3:
                _pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;
                ChargingStation_Yaw_tracking();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 3;
                docking_progress_pub->publish(docking_progress);
                break;
            case 4:
                ChargingStation_tracking2(_pRobot_Status.HOME_ID);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 4;
                docking_progress_pub->publish(docking_progress);
                break;
            case 5:
                //charging_port_On_client.call(m_request2);
                Approach_Station2Move();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 5;
                docking_progress_pub->publish(docking_progress);
                break;
            case 6:
                LED_Turn_On(9);
                //usb_cam_Off_client.call(m_request);
                RCLCPP_INFO_STREAM(nodes->get_logger(), "TETRA POSE Reset!");
                m_iReset_flag = 1;
                docking_progress.data = 6;
                docking_progress_pub->publish(docking_progress);
                ////PoseReset_call
                Reset_Robot_Pose();
                LED_Toggle_Control(1, 5,100,5,1);
                LED_Turn_On(63);

                ex_iDocking_CommandMode = 0;
                Sonar_On(0); //add_sonar_on/off

                break;
            case 9:
                printf("Docking FAIL ! \n");
                LED_Toggle_Control(1, 10,100,10,1);
                LED_Turn_On(18);
                docking_progress.data = 10;
                docking_progress_pub->publish(docking_progress);
                //ex_iDocking_CommandMode = 0;
                ex_iDocking_CommandMode = 119;
                break;
            case 10:
                Sonar_On(1); //add_sonar_on/off
                Depart_Station2Move();
                break;
            /****************************************************************/
            // Conveyor Docking Loop//
            case 11:
                LED_Toggle_Control(1, 3,100,3,100);
                LED_Turn_On(63);
                //usb_cam_On_client.call(m_request);
                sleep(2);
                ex_iDocking_CommandMode = 12;
                docking_progress.data = 1;
                docking_progress_pub->publish(docking_progress);
                break;
            case 12:
                ConveyorStation_tracking(true, _pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 2;
                docking_progress_pub->publish(docking_progress);
                break;
            case 13:
                _pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_fAR_tag_pitch;
                ConveyorStation_Yaw_tracking();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 3;
                docking_progress_pub->publish(docking_progress);
                break;
            case 14:
                ConveyorStation_tracking2(_pAR_tag_pose.m_iSelect_AR_tag_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 4;
                docking_progress_pub->publish(docking_progress);
                break;
            case 15:
                //charging_port_On_client.call(m_request2);
                Approach_Station2Move2();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    ex_iDocking_CommandMode = 0;
                }
                docking_progress.data = 5;
                docking_progress_pub->publish(docking_progress);
                break;
            case 16:
                LED_Turn_On(9);
                //usb_cam_Off_client.call(m_request);
                //ROS_INFO_STREAM("TETRA POSE Rest!");
                m_iReset_flag = 1;
                docking_progress.data = 6;
                docking_progress_pub->publish(docking_progress);
                //PoseReset_call
                Reset_Robot_Pose();
                
                LED_Toggle_Control(1, 5,100,5,1);
                LED_Turn_On(63);
                Sonar_On(0); //add_sonar_on/off
                ex_iDocking_CommandMode = 0;
                break;
            /****************************************************************/
            case 30:
                _pFlag_Value.m_bCorneringFlag = false;
                ex_iDocking_CommandMode = 0;
                break;
            case 31:
                _pFlag_Value.m_bCorneringFlag = true;
                ex_iDocking_CommandMode = 0;
                break;
            case 100: //Check Bumper
                BumperCollision_Behavior();
                break;
            case 119: //Retry Goto Home...
                printf(" Retry Goto Home ! \n");
                //costmap clear call//
                while(!clear_costmap_client->wait_for_service(1s))
                {
                    if(!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
                }
                auto result = clear_costmap_client->async_send_request(m_request);
                _pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
                _pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
                _pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
                _pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
                _pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
                _pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
                _pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;
                //TODO
                goto_goal_id.id = "HOME";
                setGoal(goal);
                _pFlag_Value.m_bflag_ComebackHome = true;
                ex_iDocking_CommandMode = 0;
                break;
            default:
                break;
        }
        //printf("ex_iDocking_CommandMode: %d \n", ex_iDocking_CommandMode);
        usleep(20000); //20ms
    }
    pthread_cancel(p_docking_thread); //Thread kill
}

/////*******************************************************************************//////
////TEST Thread Loop...
void *AutoThread_function(void *data)
{
    while(1)
    {
        if(_pFlag_Value.m_bflag_patrol)
        {
            for(int i=0; i<m_patrol_location_cnt; i++)
            {
                LED_Toggle_Control(1,3,100,3,1);
                LED_Turn_On(100); //blue led

                OpenLocationFile(arr_patrol_location[i]);
                if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
                {
                    while(!clear_costmap_client->wait_for_service(1s))
                    {
                        if(!rclcpp::ok())
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                            return;
                        }
                        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
                    }
                    auto result = clear_costmap_client->async_send_request(m_request);
                    //TODO
                    setGoal(goal);
                }
                else //Docking check...
                {
                    ex_iDocking_CommandMode = 10; //Depart Move
                    while(ex_iDocking_CommandMode != 0)
                    {
                        sleep(1); //1 sec
                        RCLCPP_INFO(nodes->get_logger(), "Depart_Station2Move....");
                    }
                }
                RCLCPP_INFO(nodes->get_logger(), "[patrol]: goto_ %s", arr_patrol_location[i].c_str());
                while(_pRobot_Status.m_iMovebase_Result != 3)
                {
                    if(!_pFlag_Value.m_bflag_patrol && !_pFlag_Value.m_bflag_goto_cancel)
                    {
                        //TODO
                        goto_goal_id.id = "";
                        RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
                        GotoCancel_pub->publish(goto_goal_id);
                        _pFlag_Value.m_bflag_goto_cancel = true;
                    }
                    else
                        usleep(1000000); //100ms
                }
                _pRobot_Status.m_iMovebase_Result = 0;
                _pFlag_Value.m_bflag_goto_cancel = false;
            }

            if(_pRobot_Status.m_iCallback_Battery <= LOW_BATTERY)
            {
                OpenLocationFile("HOME");
                if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
                {
                    //TODO
                    setGoal(goal);
                    usleep(1000000); //100ms
                    _pFlag_Value.m_bflag_ComebackHome = true;
                }
                LED_Toggle_Control(1,3,100,3,1);
                LED_Turn_On(100);
                RCLCPP_INFO(nodes->get_logger(), "[Battery Low]: come back Home~");

                _pFlag_Value.m_bflag_patrol = false;
            }
            

        }
        else if(_pFlag_Value.m_bflag_patrol2)
        {
            //Step 1. -> Goto Loading////////////////////////////////////////////////////////////
            LED_Toggle_Control(1,3,100,3,1);
            LED_Turn_On(100); //blue led

            OpenLocationFile(m_strLoading_loacation_name);
            //TODO
            goto_goal_id.id = m_strLoading_loacation_name;
            _pRobot_Status.CONVEYOR_ID = m_iLoading_ID;
            _pRobot_Status.CONVEYOR_MOVEMENT = 1;
            _pFlag_Value.m_bflag_Conveyor_docking = true;

            if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
            {
                //TODO
                setGoal(goal);
            }
            else //Docking check...
            {
                ex_iDocking_CommandMode = 10; //Depart Move
            }
            RCLCPP_INFO(nodes->get_logger(), "[patrol]: goto_conveyor-> Loading...");
            while(_pRobot_Status.m_iMovebase_Result != 3)
            {
                if(!_pFlag_Value.m_bflag_patrol2 && !_pFlag_Value.m_bflag_goto_cancel)
                {
                    //TODO
                    goto_goal_id.id = "";
                    RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
                    GotoCancel_pub->publish(goto_goal_id);
                    _pFlag_Value.m_bflag_goto_cancel = true;
                }
                else
                    usleep(1000000); //100ms
            }
            _pRobot_Status.m_iMovebase_Result = 0;
            _pFlag_Value.m_bflag_goto_cancel = false;

            while(_pRobot_Status.m_iCallback_Charging_status != 11)
            {
                if(!_pFlag_Value.m_bflag_patrol2 && !_pFlag_Value.m_bflag_goto_cancel)
                {
                    //TODO
                    goto_goal_id.id = "";
                    RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
                    GotoCancel_pub->publish(goto_goal_id);
                    _pFlag_Value.m_bflag_goto_cancel = true;
                }
                else
                    sleep(1); //1 sec
            }
            //Conveyor Movement...
            conveyor_srv->start = 1;
            while(!Conveyor_cmd_client->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO_STREAM(nodes->get_logger(), "service conveyor cmd not available, waiting again...");
            }
            auto result = Conveyor_cmd_client->async_send_request(conveyor_srv);
            RCLCPP_INFO(nodes->get_logger(), "CN1 CAll...");
            sleep(1);

            printf("_pRobot_Status.m_iConveyor_Sensor_info : %d \n", _pRobot_Status.m_iConveyor_Sensor_info );
            //Loading Finish Check
            while(_pRobot_Status.m_iConveyor_Sensor_info > 1)
            {
                RCLCPP_INFO(nodes->get_logger(), "Loading Loop Finish Wait...");
                printf("_pRobot_Status.m_iConveyor_Sensor_info : %d \n", _pRobot_Status.m_iConveyor_Sensor_info );
                sleep(1); //1 sec
            }

            sleep(5);

            //Step 2. -> Goto Unloading ////////////////////////////////////////////////////////////
            LED_Toggle_Control(1,3,100,3,1);
            LED_Turn_On(100); //blue led

            OpenLocationFile(m_strUnloading_loacation_name);
            //TODO
            goto_goal_id.id = m_strUnloading_loacation_name;
            _pRobot_Status.CONVEYOR_ID = m_iUnloading_ID;
            _pRobot_Status.CONVEYOR_MOVEMENT = 2;
            _pFlag_Value.m_bflag_Conveyor_docking = true;

            if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
            {
                //TODO
                setGoal(goal);
            }
            else //Docking check...
            {
                ex_iDocking_CommandMode = 10; //Depart Move
            }

            sleep(1);
            //Conveyor Movement...
            conveyor_srv->start = 0;
            while(!Conveyor_cmd_client->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO_STREAM(nodes->get_logger(), "service conveyor cmd not available, waiting again...");
            }
            auto result = Conveyor_cmd_client->async_send_request(conveyor_srv);

            RCLCPP_INFO(nodes->get_logger(), "[patrol]: goto_conveyor-> Unloading...");
            while(_pRobot_Status.m_iMovebase_Result != 3)
            {
                if(!_pFlag_Value.m_bflag_patrol2 && !_pFlag_Value.m_bflag_goto_cancel)
                {
                    //TODO
                    goto_goal_id.id = "";
                    RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
                    GotoCancel_pub->publish(goto_goal_id);
                    _pFlag_Value.m_bflag_goto_cancel = true;
                }
                else
                    usleep(1000000); //100ms
            }
            _pRobot_Status.m_iMovebase_Result = 0;
            _pFlag_Value.m_bflag_goto_cancel = false;

            //Conveyor Movement...
            sleep(1);
            conveyor_srv->start = 1;
            while(!Conveyor_cmd_client->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO_STREAM(nodes->get_logger(), "service conveyor cmd not available, waiting again...");
            }
            auto result = Conveyor_cmd_client->async_send_request(conveyor_srv);
            RCLCPP_INFO(nodes->get_logger(), "CN1 CAll...");

            while(_pRobot_Status.m_iCallback_Charging_status != 13)
            {
                if(!_pFlag_Value.m_bflag_patrol2 && !_pFlag_Value.m_bflag_goto_cancel)
                {
                    //TODO
                    goto_goal_id.id = "";
                    RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
                    GotoCancel_pub->publish(goto_goal_id);
                    _pFlag_Value.m_bflag_goto_cancel = true;
                }
                else
                    sleep(1); //1 sec
            }

            //Unloading Finish Check
            while(_pRobot_Status.m_iConveyor_Sensor_info <= 1)
            {
                RCLCPP_INFO(nodes->get_logger(), "Unloading Loop Finish Wait...");
                sleep(1); //1 sec
            }

            sleep(1);
            //Conveyor Movement...
            conveyor_srv->start = 0;
            while(!Conveyor_cmd_client->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO_STREAM(nodes->get_logger(), "service conveyor cmd not available, waiting again...");
            }
            auto result = Conveyor_cmd_client->async_send_request(conveyor_srv);

        }
        
        sleep(1); //1sec
    }
    pthread_cancel(p_auto_thread); //Thread2 kill
}

//add _ GUI Button callback fuction...
void RVIZ_GUI_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "STOP") //Stop...
    {
        //TODO
        goto_goal_id.id = "";
        RCLCPP_INFO(nodes->get_logger(), "Goto Cancel call");
        GotoCancel_pub->publish(goto_goal_id);
        ex_iDocking_CommandMode = 0;
    }
    
}

void RVIZ_GUI_Str_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    //msg->data.c_str();
    printf("Set Location Name: %s \n", msg->data.c_str());
    SaveLocation(msg->data.c_str(), _pTF_pose.poseTFx,_pTF_pose.poseTFy,
                                    _pTF_pose.poseTFqx,_pTF_pose.poseTFqy,_pTF_pose.poseTFqz,_pTF_pose.poseTFqw);
    
}
//TODO
void RVIZ_GUI_Goto_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    //msg->data.c_str();
    bool m_bGoto = false;
    
    if(msg->data != "") //HOME goto...
    {
        OpenLocationFile(msg->data.c_str());
        goto_goal_id.id = msg->data.c_str();
        m_bGoto = true;
    }
    else
    {

        m_bGoto = false;
    }
    
    if(m_bGoto)
    {
        RCLCPP_INFO(nodes->get_logger(), "goto_id.id: %s", goto_goal_id.id.c_str());
        //costmap clear call//
        while(!clear_costmap_client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
        }
        auto result = clear_costmap_client->async_send_request(m_request);

        if(_pRobot_Status.m_iCallback_Charging_status <= 1) //Nomal
        {
            setGoal(goal);
        }
        else //Docking...
        {
            ex_iDocking_CommandMode = 10; //Depart Move
        }

        if(goto_goal_id.id == "HOME") //HOME Point Check
        {
            _pFlag_Value.m_bflag_ComebackHome = true;
            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
            node.type = visualization_msgs::msg::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 0;
            node.action = visualization_msgs::msg::Marker::ADD; 
            node.pose.position.x = _pGoal_pose.goal_positionX;
            node.pose.position.y = _pGoal_pose.goal_positionY;
            node.pose.position.z = _pGoal_pose.goal_positionZ;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are green 
            node.color.a = 0.8; 
            node.color.r = 0.0;
            node.color.g = 0.5;
            node.color.b = 0.0;  
            node.scale.x = 0.3;
            node.scale.y = 0.3;
            node.scale.z = 0.3;

            node.lifetime = rclcpp::Duration();

            //Publish
            landmark_pub->publish(node);
        }
        else
        {
            //View _Ignition Point...
            node.header.frame_id = "/map";
            node.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
            node.type = visualization_msgs::msg::Marker::SPHERE;
            node.ns = "Ignition_shapes";
            node.id = 0;
            node.action = visualization_msgs::msg::Marker::ADD; 
            node.pose.position.x = _pGoal_pose.goal_positionX;
            node.pose.position.y = _pGoal_pose.goal_positionY;
            node.pose.position.z = _pGoal_pose.goal_positionZ;
            
            node.pose.orientation.x = 0.0;
            node.pose.orientation.y = 0.0; 
            node.pose.orientation.z = 0.0; 
            node.pose.orientation.w = 1.0; 
            
            // Points are green 
            node.color.a = 0.8; 
            node.color.r = 1.0;
            node.color.g = 0.0;
            node.color.b = 0.0;  
            node.scale.x = 0.3;
            node.scale.y = 0.3;
            node.scale.z = 0.3;

            node.lifetime = rclcpp::Duration();

            //Publish
            landmark_pub->publish(node);
        }

        m_bGoto = false; //reset
    }
    
    
}

void Reset_Call_service()
{
	_pFlag_Value.m_bFlag_nomotion = false;

	//IMU reset//
    while(!euler_angle_reset_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service euler angle reset cmd not available, waiting again...");
    }
    auto result = euler_angle_reset_cmd_client->async_send_request(euler_angle_reset_srv);
	printf("## IMU Reset ! \n");
	//tetra odometry Reset//
	tetra_PoseRest.data = m_iReset_flag;
	PoseReset_pub->publish(tetra_PoseRest);
	usleep(100000);

    	//robot_localization::SetPose ekf_reset;
	setpose_srv->pose.header.frame_id = tf_prefix_ + "/odom";
	setpose_srv->pose.header.stamp = nodes->get_clock()->now(); //ros::Time::now();

	setpose_srv->pose.pose.pose.position.x = _pReset_srv.init_position_x;
	setpose_srv->pose.pose.pose.position.y = _pReset_srv.init_position_y;
	setpose_srv->pose.pose.pose.position.z = _pReset_srv.init_position_z;

	setpose_srv->pose.pose.pose.orientation.x = _pReset_srv.init_orientation_x;
	setpose_srv->pose.pose.pose.orientation.y = _pReset_srv.init_orientation_y;
	setpose_srv->pose.pose.pose.orientation.z = _pReset_srv.init_orientation_z;
	setpose_srv->pose.pose.pose.orientation.w = _pReset_srv.init_orientation_w;

	setpose_srv->pose.pose.covariance[0] = 0.25;
	setpose_srv->pose.pose.covariance[6 * 1 + 1] = 0.25;
	setpose_srv->pose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

    while(!SetPose_cmd_client->wait_for_service(1s))
    {
        if(!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service set pose cmd not available, waiting again...");
    }
    auto result = SetPose_cmd_client->async_send_request(setpose_srv); //Set_pose call//
	printf("##Set_Pose(EKF)2! \n");

	initPose_.header.stamp = nodes->get_clock()->now(); //ros::Time::now(); 
	initPose_.header.frame_id = "map";
	//position
	initPose_.pose.pose.position.x = _pReset_srv.init_position_x;
	initPose_.pose.pose.position.y = _pReset_srv.init_position_y;
	initPose_.pose.pose.position.z = _pReset_srv.init_position_z;
	//orientation
	initPose_.pose.pose.orientation.x = _pReset_srv.init_orientation_x;
	initPose_.pose.pose.orientation.y = _pReset_srv.init_orientation_y;
	initPose_.pose.pose.orientation.z = _pReset_srv.init_orientation_z;
	initPose_.pose.pose.orientation.w = _pReset_srv.init_orientation_w;

	initPose_.pose.covariance[0] = 0.25;
	initPose_.pose.covariance[6 * 1 + 1] = 0.25;
	initPose_.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

	initialpose_pub->publish(initPose_);
	printf("##Set_initPose(2D Estimate)2! \n");

    //usleep(500000);

	_pFlag_Value.m_bFlag_nomotion = true;
}

bool SetEKF_Command(const std::shared_ptr<tetra_msgs::srv::Setekf::Request> req, 
				    std::shared_ptr<tetra_msgs::srv::Setekf::Response> res)
{   
	bool bResult = false;

	_pReset_srv.init_position_x = req->init_position_x;
	_pReset_srv.init_position_y = req->init_position_y;
	_pReset_srv.init_position_z = req->init_position_z;

	_pReset_srv.init_orientation_x = req->init_orientation_x;
	_pReset_srv.init_orientation_y = req->init_orientation_y;
	_pReset_srv.init_orientation_z = req->init_orientation_z;
	_pReset_srv.init_orientation_w = req->init_orientation_w;

	_pReset_srv.bflag_reset = true;

	printf("[SetEKF_Command]: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f \n", _pReset_srv.init_position_x,_pReset_srv.init_position_y,_pReset_srv.init_position_z,
	_pReset_srv.init_orientation_x, _pReset_srv.init_orientation_y, _pReset_srv.init_orientation_z, _pReset_srv.init_orientation_w);

	//Reset_Call_service();

	bResult = true;
	res->command_result = bResult;
	return true;
}

//InitialposeCallback
void InitialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msgInitialpose)
{
  //printf("## InitialposeCallback ## \n");
  _pFlag_Value.m_bFlag_Initialpose = true;
}

/////*******************************************************************************//////

int main (int argc, char** argv)
{
  signal(SIGINT,my_handler);

  rclcpp::init(argc, argv);
  nodes = rclcpp::Node::make_shared("tetra_landmark_save");
  cmdpub_ = nodes->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",100);
  auto cmdsub_ = nodes->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 100, &cmd_vel_Callback);
  //Servo On/Off publish
  servo_pub = nodes->create_publisher<std_msgs::msg::Int32>("Servo_ON",10);
  //TODO
  service_pub = nodes->create_publisher<move_base_msgs::msg::MoveBaseActionGoal>("move_base/goal", 10);
  auto sub_amcl = nodes->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 100 ,&poseAMCLCallback);
  //Navigation Result//
  //TODO
  auto result_sub = nodes->create_subscription<move_base_msgs::msg::MoveBaseActionResult>("move_base/result", 10, &resultCallback);
  //TODO
  auto status_sub = nodes->create_subscription<actionlib_msgs::msg::GoalStatusArray>("move_base/status", 10, &statusCallback); //add...
  //Navigation Cancel//
  //TODO
  GotoCancel_pub = nodes->create_publisher<actionlib_msgs::msg::GoalID>("move_base/cancel",10);
  //PoseReset//
  PoseReset_pub = nodes->create_publisher<std_msgs::msg::Int32>("PoseRest",10);
  //Acceleration input//
  Accel_pub = nodes->create_publisher<std_msgs::msg::Int32>("accel_vel",10);
  //AR_TAG_subscriber//
  auto AR_sub = nodes->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>("ar_pose_marker", 100, &AR_tagCallback);
  //Laser Scan subscriber//
  auto scan_sub = nodes->create_subscription<sensor_msgs::msg::LaserScan>("scan", 100, &LaserScanCallback);
  //Depthimage to scan subscriber//
  auto pcl1_sub = nodes->create_subscription<sensor_msgs::msg::LaserScan>("pcl_1", 100, &PCL1_Callback);
  auto pcl2_sub = nodes->create_subscription<sensor_msgs::msg::LaserScan>("pcl_2", 100, &PCL2_Callback);
  //virtual costmap
  virtual_obstacle_pub = nodes->create_publisher<tetra_msgs::msg::Obstacles>("virtual_costamp_layer/obsctacles", 100);
  virtual_obstacle2_pub = nodes->create_publisher<tetra_msgs::msg::Obstacles2>("virtual_costamp_layer2/obsctacles", 100);
  //amcl particlecloud Subscribe
  auto pacticle_sub = nodes->create_subscription<geometry_msgs::msg::PoseArray>("particlecloud", 3000, &Particle_Callback);
  //teb Markers Subscribe
  //TODO
  auto tebmarksers_sub = nodes->create_subscription<visualization_msgs::msg::Marker>("move_base/TebLocalPlannerROS/teb_markers", 100, &TebMarkers_Callback);
  //teb_localPlan Subscribe
  //TODO
  auto teblocalplan_sub = nodes->create_subscription<geometry_msgs::msg::PoseArray>("move_base/TebLocalPlannerROS/teb_poses", 100, &Teblocalplan_Callback);
  //Initialpose publish
  initialpose_pub = nodes->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 100);
  //Initialpose Subscribe
  auto sub_initialpose = nodes->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 100, &InitialposeCallback);
  //Joystick//
  auto joy_sub = nodes->create_subscription<sensor_msgs::msg::Joy>("joy", 10, &joyCallback);
  //Read HOME Docking ID Param Read//
  nodes->declare_parameter("HOME_ID", 0);
  _pRobot_Status.HOME_ID = nodes->get_parameter("HOME_ID").as_int();
  printf("##HOME_ID: %d \n", _pRobot_Status.HOME_ID);
  //add Bumper to Behavior
  nodes->declare_parameter("bumper_behavior", false);
  _pFlag_Value.BUMPER_BT = nodes->get_parameter("bumper_behavior").as_bool();
  printf("##bumper_behavior: %d \n", _pFlag_Value.BUMPER_BT);
  //Read Max_linear_Velocity
  nodes->declare_parameter("max_vel_x", 0.0);
  _pDynamic_param.MAX_Linear_velocity = nodes->get_parameter("max_vel_x").as_double();
  printf("##max_vel_x: %f \n", _pDynamic_param.MAX_Linear_velocity);
  node->declare_parameter("tf_prefix", "");
  tf_prefix_ = node->get_parameter("tf_prefix").as_string();

  //add GUI...
  auto GUI_sub = nodes->create_subscription<std_msgs::msg::String>("/rviz_visual_tools_gui_btn", 10, &RVIZ_GUI_Callback);
  auto GUI_Str_sub = nodes->create_subscription<std_msgs::msg::String>("/rviz_visual_tools_gui_location_name", 10, &RVIZ_GUI_Str_Callback);
  auto GUI_goto_sub = nodes->create_subscription<std_msgs::msg::String>("/rviz_visual_tools_gui_goto_location_name", 10, &RVIZ_GUI_Goto_Callback);

  landmark_pub = nodes->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

  //Command Service//
  getlocation_service = node->create_service<tetra_msgs::srv::Getlocation>("getlocation_cmd", &GetLocation_Command);
  goto_service = node->create_service<tetra_msgs::srv::Gotolocation>("goto_cmd", &Goto_Command);
  goto_service2 = node->create_service<tetra_msgs::srv::Gotolocation2>("goto_cmd2", &Goto_Command2);
  setlocation_service = node->create_service<tetra_msgs::srv::Setlocation>("setlocation_cmd", &SetLocation_Command);
  save_map_service = node->create_service<tetra_msgs::srv::Setsavemap>("savemap_cmd", &SetSavemap_Command);
  getinfo_service = node->create_service<tetra_msgs::srv::Getinformation>("getinfo_cmd", &GetInformation_Command);
  docking_service = node->create_service<tetra_msgs::srv::Dockingcontrol>("docking_cmd", &Docking_Command);
  locationlist_service = node->create_service<tetra_msgs::srv::Getlocationlist>("locationlist_cmd", &LocationList_Command);
  delete_location_service = node->create_service<tetra_msgs::srv::Deletelocation>("delete_location_cmd", &DeleteLocation_Command);
  //Land mark Service//
  landmarklist_service = node->create_service<tetra_msgs::srv::Getlandmarklist>("landmarklist_cmd", &LandmarkList_Command);
  delete_landmark_service = node->create_service<tetra_msgs::srv::Deletelandmark>("delete_landmark_cmd", &DeleteLandmark_Command);
  //Map Service//
  maplist_service = node->create_service<tetra_msgs::srv::Getmaplist>("maplist_cmd", &MapList_Command);
  delete_map_service = node->create_service<tetra_msgs::srv::Deletemap>("delete_map_cmd", &DeleteMap_Command);
  gotocancel_service = node->create_service<tetra_msgs::srv::Gotocancel>("gotocancel_cmd", &GotoCancel_Command);
  sloptime_service = node->create_service<tetra_msgs::srv::Accelerationslop>("sloptime_cmd", &SlopTime_Command);
  servo_service = node->create_service<tetra_msgs::srv::Servo>("servo_cmd", &Servo_Command);
  //Docking Exit Service//
  docking_exit = node->create_service<std_srvs::srv::Empty>("docking_Stop", &DockingStop_Command);
  //Dynamic reconfigure Service//
  setspeed_service = node->create_service<tetra_msgs::srv::Setmaxspeed>("setspeed_cmd", &Setspeed_Command);
  //rosrun & roslaunch command//
  mapping_service = node->create_service<tetra_msgs::srv::Runmapping>("mapping_cmd", &mapping_Command);
  navigation_service = node->create_service<tetra_msgs::srv::Runnavigation>("navigation_cmd", &navigation_Command);
  nodekill_service = node->create_service<tetra_msgs::srv::Rosnodekill>("nodekill_cmd", &nodekill_Command);
  //set initPose command//
  setinitpose_service = node->create_service<tetra_msgs::srv::Setinitpose>("setinitpose_cmd", &SetInitPose_Command);
  //set 2D_Pose_Estimate command//
  pose_Estimate_service = node->create_service<tetra_msgs::srv::PoseEstimate>("pose_estimate_cmd", &Set2D_Pose_Estimate_Command);
  //Convetor Service//
  gotoconveyor_service = node->create_service<tetra_msgs::srv::Gotoconveyor>("gotoconveyor_cmd", &Goto_Conveyor_Command);
  loadingcheck_service = node->create_service<tetra_msgs::srv::Loadingcheck>("loadingcheck_service_cmd", &Loading_check_Command);
  unloadingcheck_service = node->create_service<tetra_msgs::srv::Unloadingcheck>("unloadingcheck_service_cmd", &Unloading_check_Command);
  //Patrol Service//
  patrol_service = node->create_service<tetra_msgs::srv::Patrol>("patrol_cmd", &Patrol_Command);
  patrol_conveyor_service = node->create_service<tetra_msgs::srv::PatrolConveyor>("patrol_conveyor_cmd", &Patrol_Conveyor_Command);
  //Delete Data All Service//
  deletedataall_service = node->create_service<tetra_msgs::srv::Deletedataall>("deletedataall_cmd", &DeleteData_All_Command);
  //Virtual costmap Service//
  virtual_obstacle_service = node->create_service<tetra_msgs::srv::VirtualObstacle>("virtual_obstacle_cmd", &Virtual_Obstacle_Command);
  //Set EKF & IMU Reset Service//
  set_ekf_service = node->create_service<tetra_msgs::srv::Setekf>("set_ekf_cmd", &SetEKF_Command);
  
  //usb_cam Service Client...
  usb_cam_On_client = nodes->create_client<std_srvs::srv::Empty>("usb_cam/start_capture");
  usb_cam_Off_client = nodes->create_client<std_srvs::srv::Empty>("usb_cam/stop_capture");
  //Charging Port Service Client...
  charging_port_On_client = nodes->create_client<std_srvs::srv::Empty>("charging_port_on");
  charging_port_Off_client = nodes->create_client<std_srvs::srv::Empty>("charging_port_off");
  //request_nomotion_update Service Client
  request_nomotion_update_client = nodes->create_client<std_srvs::srv::Empty>("request_nomotion_update");
  //LED Control Client//
  led_cmd_client = nodes->create_client<tetra_msgs::srv::Ledcontrol>("led_cmd");
  ledtoggle_cmd_client = nodes->create_client<tetra_msgs::srv::Ledtogglecontrol>("ledtoggle_cmd");
  turnon_cmd_client = nodes->create_client<tetra_msgs::srv::Toggleon>("turnon_cmd");
  //Clear costmaps//
  clear_costmap_client = nodes->create_client<std_srvs::Empty>("move_base/clear_costmaps");
  //Conveyor Move Client
  Conveyor_cmd_client = nodes->create_client<tetra_msgs::srv::ConveyorAutoMovement>("Auto_Move_cmd");
  //IMU Service Client//
  euler_angle_reset_cmd_client = nodes->create_client<tetra_msgs::srv::EulerAngleReset>("euler_angle_reset_cmd");
  //robot_localization Service Client//
  SetPose_cmd_client = nodes->create_client<tetra_msgs::srv::SetPose>("set_pose");
  //sonar sensor on/off
  power_sonar_cmd_client = nodes->create_client<tetra_msgs::srv::PowerSonarCmd>("Power_sonar_start_cmd");

  //Infomation_subscriber//
  auto tetra_battery = nodes->create_subscription<std_msgs::msg::Int32>("tetra_battery", 1, &BatteryCallback);
  auto emg_state = nodes->create_subscription<std_msgs::msg::Int32>("emg_state", 1, &EMGCallback);
  auto bumper_data = nodes->create_subscription<std_msgs::msg::Int32>("bumper_data", 1, &BumperCallback);
  auto docking_status = nodes->create_subscription<std_msgs::msg::Int32>("docking_status", 1, &ChargingCallback);
  //Conveyor_Info Subscriber//
  auto loadcell_status = nodes->create_subscription<std_msgs::msg::Float64>("conveyor_loadcell", 1, &LoadcellCallback);
  auto sensor_status = nodes->create_subscription<std_msgs::msg::Int32>("conveyor_sensor", 1, &SensorCallback);

  //Ultrasonic_subscriber//
  auto ultrasonic_FL = nodes->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_D_L", 10, &Ultrasonic_DL_Callback);
  auto ultrasonic_FR = nodes->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_D_R", 10, &Ultrasonic_DR_Callback);
  auto ultrasonic_RL = nodes->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_R_L", 10, &Ultrasonic_RL_Callback);
  auto ultrasonic_RR = nodes->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_R_R", 10, &Ultrasonic_RR_Callback);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //bumper_data to Pointcloud2_data///
  pointcloud_.header.frame_id = tf_prefix_ + "/Front_bumper";
  pointcloud_.width  = 3;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);
  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";
  int offset = 0; 
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
      pointcloud_.fields[d].count    = 1;
      pointcloud_.fields[d].offset   = offset;
      pointcloud_.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }
  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;
  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));
  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  pointcloud_pub_ = nodes->create_publisher<sensor_msgs::msg::PointCloud2> ("bumper_pointcloud", 100);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Docking Loop 
  docking_progress.data = 0;
  docking_progress_pub = nodes->create_publisher<std_msgs::msg::Int32>("docking_progress", 1);
  //Docking positioning publish
  positioning_pub = nodes->create_publisher<geometry_msgs::msg::Pose2D>("positioning", 10);
  
  /*Thread Create...*/
  int docking_thread_id, auto_thread_id;
  int a = 1;
  int b = 1;
  docking_thread_id = pthread_create(&p_docking_thread, NULL, DockingThread_function, (void *)&a);
  if (docking_thread_id < 0)
  {
      printf("docking thread create error !!");
      exit(0);
  }
  // auto_thread_id = pthread_create(&p_auto_thread, NULL, AutoThread_function, (void *)&b);
  // if (auto_thread_id < 0)
  // {
  //     printf("auto thread create error !!");
  //     exit(0);
  // }  

  //TF transform//
  auto tf_buffer_1 = std::make_unique<tf2_ros::Buffer>(nodes->get_clock());
  auto tf_listener_1 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1);
  auto tf_buffer_2 = std::make_unique<tf2_ros::Buffer>(nodes->get_clock());
  auto tf_listener_2 = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2);

  rclcpp::Rate loop_rate(30.0); //default: 30HZ

  LED_Toggle_Control(1, 3,100,3,1);
  LED_Turn_On(63);

  int iCheckCnt = 0;
  //Robot IP Check//
  m_strRobotIP = GetWIFI_IPAddress();
  printf("###[Robot IP]: %s ###\n", m_strRobotIP.c_str());
  printf("HOME_strLOCATION: %s \n", _pHomePose.HOME_strLOCATION.c_str());
  printf("HOME_dPOSITION_X: %f \n", _pHomePose.HOME_dPOSITION_X);
  printf("HOME_dPOSITION_Y: %f \n", _pHomePose.HOME_dPOSITION_Y);
  printf("HOME_dPOSITION_Z: %f \n", _pHomePose.HOME_dPOSITION_Z);
  printf("HOME_dQUATERNION_X: %f \n", _pHomePose.HOME_dQUATERNION_X);
  printf("HOME_dQUATERNION_Y: %f \n", _pHomePose.HOME_dQUATERNION_Y);
  printf("HOME_dQUATERNION_Z: %f \n", _pHomePose.HOME_dQUATERNION_Z);
  printf("HOME_dQUATERNION_W: %f \n", _pHomePose.HOME_dQUATERNION_W);
  
  std::string node_name = "/" + tf_prefix_ + "/nav2_controller"; // add nav2 Die Check node_name
  node->declare_parameter("active_map", false);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(nodes);

    // add move_base Die Check Loop
    m_bActive_map_check = node->get_parameter("active_map").as_bool();
    if(m_bActive_map_check)
    {
      if(checkNode(node_name) == true)
      {
        // ROS_ERROR("move_base alive");
      }   
      else
      {
        // ROS_ERROR("move_base die");
      }
    }

    if(_pFlag_Value.m_bFlag_Obstacle_Center || m_iViaPoint_Index <= 1)
    {
      if(!m_flag_Dynamic_Linear_velocity_major_update)
      {
        Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", _pDynamic_param.MAX_Linear_velocity / 2.0);
        m_flag_Dynamic_Linear_velocity_major_update = true;
        m_flag_Dynamic_Linear_velocity_minor_update = false;
        _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
      }
      else
      {
        _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
      }
    }
    else
    {
      if(!_pFlag_Value.m_bTebMarker_reconfigure_flag)
      {
        if(!m_flag_Dynamic_Linear_velocity_minor_update)
        {
          Dynamic_reconfigure_Teb_Set_DoubleParam("max_vel_x", _pDynamic_param.MAX_Linear_velocity);
          m_flag_Dynamic_Linear_velocity_minor_update = true;
          m_flag_Dynamic_Linear_velocity_major_update = false;
          _pFlag_Value.m_bTebMarker_reconfigure_flag = true;
        }
      }
      else
      {
        _pFlag_Value.m_bTebMarker_reconfigure_flag = false;
      }
    }
    //IMU Reset Loop//
    if(m_iTimer_cnt >= 500) //10 sec_polling
    {
      //costmap clear call//
      while(!clear_costmap_client->wait_for_service(1s))
      {
        if(!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO_STREAM(nodes->get_logger(), "service clear costmap not available, waiting again...");
      }
      auto result = clear_costmap_client->async_send_request(m_request);
      m_iTimer_cnt = 0;
      Reset_Robot_Pose();
      //ROS_INFO("Reset_Robot_Pose Call !");
    }
    else
    {
      if(_pRobot_Status.m_iCallback_Charging_status == 2 || _pRobot_Status.m_iCallback_Charging_status == 3 || _pRobot_Status.m_iCallback_Charging_status == 6 || _pRobot_Status.m_iCallback_Charging_status == 7)
      {
        m_iTimer_cnt ++;
      }
      else
      {
        m_iTimer_cnt = 0;
      }
    }
    //Reset service call check//
    if(_pReset_srv.bflag_reset)
    {
      Reset_Call_service();
      _pReset_srv.bflag_reset = false;
    }
    if(_pRobot_Status.m_iCallback_Charging_status == 1)
    {
      //Get Active map param..//
      m_bActive_map_check = node->get_parameter("active_map").as_bool();
      if(m_bActive_map_check)
      {
        //map to base_footprint TF Pose////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        geometry_msgs::msg::TransformStamped ts_msg;
        try
        {
          ts_msg = tf_buffer_1->lookupTransform("/map", tf_prefix_ + "/base_footprint", tf2::TimePointZero);

          _pTF_pose.poseTFx = ts_msg.transform.translation.x;
          _pTF_pose.poseTFy = ts_msg.transform.translation.y;
          _pTF_pose.poseTFz = ts_msg.transform.translation.z;
          _pTF_pose.poseTFqx = ts_msg.transform.rotation.x;
          _pTF_pose.poseTFqy = ts_msg.transform.rotation.y;
          _pTF_pose.poseTFqz = ts_msg.transform.rotation.z;
          _pTF_pose.poseTFqw = ts_msg.transform.rotation.w;

        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_INFO(nodes->get_logger(), "[TF_Transform_Error(map to base_footprint)]: %s", ex.what());
          continue;
        }

        //map to odom TF Pose////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        geometry_msgs::msg::TransformStamped ts_msg2;
        bool bCheck_waitForTransform = false;
        try
        {
          bCheck_waitForTransform = true;
          ts_msg2 = tf_buffer_2->lookupTransform("/map", tf_prefix_ + "/odom", tf2::TimePointZero);

          _pTF_pose2.poseTFx2 = ts_msg2.transform.translation.x;
          _pTF_pose2.poseTFy2 = ts_msg2.transform.translation.y;
          _pTF_pose2.poseTFz2 = ts_msg2.transform.translation.z;
          _pTF_pose2.poseTFqx2 = ts_msg2.transform.rotation.x;
          _pTF_pose2.poseTFqy2 = ts_msg2.transform.rotation.y;
          _pTF_pose2.poseTFqz2 = ts_msg2.transform.rotation.z;
          _pTF_pose2.poseTFqw2 = ts_msg2.transform.rotation.w;

          //itself call client service loop
          m_dTF_Yaw = Quaternion2Yaw_rad(_pTF_pose2.poseTFqw2, _pTF_pose2.poseTFqx2, _pTF_pose2.poseTFqy2, _pTF_pose2.poseTFqz2);
          m_dTF_New_Pose_X = (((_pTF_pose2.poseTFx2 * cos(m_dTF_Yaw)) + (_pTF_pose2.poseTFy2 * sin(m_dTF_Yaw))));
          m_dTF_New_Pose_Y = (((_pTF_pose2.poseTFx2 * -sin(m_dTF_Yaw)) + (_pTF_pose2.poseTFy2 * cos(m_dTF_Yaw))));
        }
        catch (const tf2::TransformException &ex2)
        {
          bCheck_waitForTransform = false;
          ROS_ERROR("[TF_Transform_Error2(map to odom)]: %s", ex2.what());
          continue;
        }
        if(bCheck_waitForTransform)
        {
          m_iList_Count = virtual_obstacle.list.size();
          if(m_iList_Count > 0)
          {
            if(m_bFlag_nomotion_call || !_pFlag_Value.m_bFlag_nomotion || m_flag_Dynamic_reconfigure_call || m_flag_setgoal || _pFlag_Value.m_bTebMarker_reconfigure_flag)
            {
              loop_rate.sleep();
              continue;
            }

            //message copy...
            virtual_obstacle2.list.clear();
            virtual_obstacle2.list.resize(m_iList_Count);
            m_iList_Count2 = virtual_obstacle2.list.size();
            if(m_iList_Count2 > 0)
            {
              for(int i=0; i<m_iList_Count2; i++)
              {
                m_iMode_Count = virtual_obstacle.list[i].form.size();
                //virtual_obstacle2.list[i].form.clear();
                virtual_obstacle2.list[i].form.resize(m_iMode_Count);
                m_iMode_Count2 = virtual_obstacle2.list[i].form.size();
                if(m_iMode_Count2 > 0)
                {
                  for(int j=0; j<m_iMode_Count; j++)
                  {
                    virtual_obstacle2.list[i].form[j].x = floor(((((virtual_obstacle.list[i].form[j].x *  cos(m_dTF_Yaw)) + (virtual_obstacle.list[i].form[j].y * sin(m_dTF_Yaw)))) - m_dTF_New_Pose_X)*1000.f + 0.5) /1000.f;
                    virtual_obstacle2.list[i].form[j].y = floor(((((virtual_obstacle.list[i].form[j].x * -sin(m_dTF_Yaw)) + (virtual_obstacle.list[i].form[j].y  * cos(m_dTF_Yaw)))) - m_dTF_New_Pose_Y)*1000.f + 0.5) /1000.f;
                    virtual_obstacle2.list[i].form[j].z = floor((virtual_obstacle.list[i].form[j].z)*1000.f + 0.5) /1000.f;
                  }
                }
              }
              if(m_bFlag_nomotion_call || !_pFlag_Value.m_bFlag_nomotion || m_flag_Dynamic_reconfigure_call || m_flag_setgoal || _pFlag_Value.m_bTebMarker_reconfigure_flag)
              {
                loop_rate.sleep();
                continue;
              }
              virtual_obstacle2_pub->publish(virtual_obstacle2);
            }
          }
        }
        else
        {
          printf("[Error]listener2.waitForTransform Fail & lookupTransform Fail!! \n");
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      }
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
