#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp" 

//Service_srv file
#include "tetra_msgs/srv/gotolocation.hpp" //SRV
#include "tetra_msgs/srv/gotolocation2.hpp" //SRV
#include "tetra_msgs/srv/gotocancel.hpp" //SRV
#include "tetra_msgs/srv/getlocation.hpp" //SRV
#include "tetra_msgs/srv/setlocation.hpp" //SRV
#include "tetra_msgs/srv/setsavemap.hpp" //SRV
#include "tetra_msgs/srv/getlocationlist.hpp" //SRV
#include "tetra_msgs/srv/deletelocation.hpp" //SRV
#include "tetra_msgs/srv/runmapping.hpp" //SRV
#include "tetra_msgs/srv/runnavigation.hpp" //SRV
#include "tetra_msgs/srv/rosnodekill.hpp" //SRV
#include "tetra_msgs/srv/getmaplist.hpp" //SRV
#include "tetra_msgs/srv/setmaxspeed.hpp" //SRV
#include "tetra_msgs/srv/dockingcontrol.hpp" //SRV
#include "tetra_msgs/srv/power_set_outport.hpp" //SRV
#include "tetra_msgs/srv/power_get_io_status.hpp" //SRV
//add...230405_wbjin
#include "tetra_msgs/srv/power_set_single_outport.hpp" //SRV

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <iostream>
#include <thread>
#include <dirent.h>
#include <signal.h>

#define BUF_LEN 4096
using namespace std;

char buffer[BUF_LEN];
char Send_buffer[BUF_LEN];
struct sockaddr_in server_addr, client_addr;
char temp[20];
int server_fd, client_fd;
int len, msg_size;
pthread_t p_auto_thread;
pthread_t p_auto_thread2;
bool m_bflag_thread = false;

///*Socket Data define *///
char cSTX[] = "DS";
char cETX[] = "XX";
char* m_cSTX;
char* m_cLEN;
char* m_cMOD;
char* m_cCMD;
char* m_cPARAM[16];
char* m_cDATA;
char* m_cETX;
char m_cCOMMA[] = ",";
int m_iLEN = 0;
int m_iMOD = 0;

//data///
int m_iTotal_Map_cnt = 0;
int m_iTotal_Location_cnt = 0;
string m_strMap[100] = {"", }; //Max Map data
string m_strLocation[255] = {"", }; //Max Location data

//ros Node
std::shared_ptr<rclcpp::Node> nodes;

//*Custom Service Client*//
rclcpp::Client<tetra_msgs::srv::Gotolocation>::SharedPtr goto_cmd_client;
auto goto_cmd_service = std::make_shared<tetra_msgs::srv::Gotolocation::Request>();
rclcpp::Client<tetra_msgs::srv::Gotolocation2>::SharedPtr goto_cmd_client2;
auto goto_cmd_service2 = std::make_shared<tetra_msgs::srv::Gotolocation2::Request>();
rclcpp::Client<tetra_msgs::srv::Gotocancel>::SharedPtr gotocancel_cmd_client;
auto gotocancel_cmd_service = std::make_shared<tetra_msgs::srv::Gotocancel::Request>();
rclcpp::Client<tetra_msgs::srv::Setlocation>::SharedPtr setlocation_cmd_client;
auto setlocation_cmd_service = std::make_shared<tetra_msgs::srv::Setlocation::Request>();
rclcpp::Client<tetra_msgs::srv::Getlocation>::SharedPtr getlocation_cmd_client;
auto getlocation_cmd_service = std::make_shared<tetra_msgs::srv::Getlocation::Request>();
rclcpp::Client<tetra_msgs::srv::Getlocationlist>::SharedPtr locationlist_cmd_client;
auto locationlist_cmd_service = std::make_shared<tetra_msgs::srv::Getlocationlist::Request>();
rclcpp::Client<tetra_msgs::srv::Getmaplist>::SharedPtr maplist_cmd_client;
auto maplist_cmd_service = std::make_shared<tetra_msgs::srv::Getmaplist::Request>();
rclcpp::Client<tetra_msgs::srv::Setmaxspeed>::SharedPtr setspeed_cmd_client;
auto setspeed_cmd_service = std::make_shared<tetra_msgs::srv::Setmaxspeed::Request>();
rclcpp::Client<tetra_msgs::srv::Runnavigation>::SharedPtr navigation_cmd_client;
auto navigation_cmd_service = std::make_shared<tetra_msgs::srv::Runnavigation::Request>();
rclcpp::Client<tetra_msgs::srv::Runmapping>::SharedPtr mapping_cmd_client;
auto mapping_cmd_service = std::make_shared<tetra_msgs::srv::Runmapping::Request>();
rclcpp::Client<tetra_msgs::srv::Setsavemap>::SharedPtr mapsave_cmd_client;
auto mapsave_cmd_service = std::make_shared<tetra_msgs::srv::Setsavemap::Request>();
rclcpp::Client<tetra_msgs::srv::Rosnodekill>::SharedPtr nodekill_cmd_client;
auto nodekill_cmd_service = std::make_shared<tetra_msgs::srv::Rosnodekill::Request>();
rclcpp::Client<tetra_msgs::srv::Deletelocation>::SharedPtr deletelocation_cmd_client;
auto deletelocation_cmd_service = std::make_shared<tetra_msgs::srv::Deletelocation::Request>();
rclcpp::Client<tetra_msgs::srv::Dockingcontrol>::SharedPtr dockingcontrol_cmd_client;
auto dockingcontrol_cmd_service = std::make_shared<tetra_msgs::srv::Dockingcontrol::Request>();
rclcpp::Client<tetra_msgs::srv::PowerSetOutport>::SharedPtr output_cmd_client;
auto output_cmd_service = std::make_shared<tetra_msgs::srv::PowerSetOutport::Request>();
rclcpp::Client<tetra_msgs::srv::PowerGetIoStatus>::SharedPtr gpio_status_cmd_client;
auto gpio_status_cmd_service = std::make_shared<tetra_msgs::srv::PowerGetIoStatus::Request>();
//add_230405
rclcpp::Client<tetra_msgs::srv::PowerSetSingleOutport>::SharedPtr single_output_cmd_client;
auto single_output_cmd_service = std::make_shared<tetra_msgs::srv::PowerSetSingleOutport::Request>();

//***************************************************************************************************************************************/
//Struct define//
typedef struct ODOMETRY
{
  double dOdom_position_x = 0.0;
  double dOdom_position_y = 0.0;
  double dOdom_position_z = 0.0;
  double dOdom_quaternion_x = 0.0;
  double dOdom_quaternion_y = 0.0;
  double dOdom_quaternion_z = 0.0;
  double dOdom_quaternion_w = 0.0;
  double dTwist_linear = 0.0;
  double dTwist_angular = 0.0;

}ODOMETRY;
ODOMETRY _pOdometry;

typedef struct AMCL_POSE
{
  double dPoseAMCLx = 0.0;
  double dPoseAMCLy = 0.0;
  double dPoseAMCLz = 0.0;
  double dPoseAMCLqx = 0.0;
  double dPoseAMCLqy = 0.0;
  double dPoseAMCLqz = 0.0;
  double dPoseAMCLqw = 0.0;

}AMCL_POSE;
AMCL_POSE _pAMCL_pose;

typedef struct GOAL_POSE
{
  double dGoal_positionX = 0.0;
  double dGoal_positionY = 0.0;
  double dGoal_positionZ = 0.0;
  double dGoal_quarterX = 0.0;
  double dGoal_quarterY = 0.0;
  double dGoal_quarterZ = 0.0;
  double dOdom_position_z = 0.0;
  double dOdom_quaternion_x = 0.0;
  double dOdom_quaternion_y = 0.0;
  double dOdom_quaternion_z = 0.0;
  double dOdom_quaternion_w = 0.0;
  double dTwist_linear = 0.0;
  double dTwist_angular = 0.0;
  int iCallback_ErrorCode = 0;
  int iCallback_EMG = 0;
  int iCallback_Bumper = 0;
  int iCallback_Charging_status = 0;
  //Bumper Collision Behavior//
  int iBumperCollisionBehavor_cnt = 0;
  //auto test
  int iMovebase_Result = 0;
  //Conveyor Info..(Option)
  double dLoadcell_weight = 0.0;
  int iConveyor_Sensor_info = 0;
  int HOME_ID = 0; //Docking ID Param Read//
  int CONVEYOR_ID = 0;
  int CONVEYOR_MOVEMENT = 0; // 0: nomal , 1: Loading , 2: Unloading

}ROBOT_STATUS;
ROBOT_STATUS _pRobot_Status;

//***************************************************************************************************************************************/
//Callback Function///
void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  _pOdometry.dOdom_position_x = msg->pose.pose.position.x;
  _pOdometry.dOdom_position_y = msg->pose.pose.position.y;
  _pOdometry.dOdom_position_z = msg->pose.pose.position.z;
  _pOdometry.dOdom_quaternion_x = msg->pose.pose.orientation.x;
  _pOdometry.dOdom_quaternion_y = msg->pose.pose.orientation.y;
  _pOdometry.dOdom_quaternion_z = msg->pose.pose.orientation.z;
  _pOdometry.dOdom_quaternion_w = msg->pose.pose.orientation.w;

  _pOdometry.dTwist_linear = msg->twist.twist.linear.x;
  _pOdometry.dTwist_angular = msg->twist.twist.angular.z;
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
  // time_t curr_time;
  // struct tm *curr_tm;
  // curr_time = time(NULL);
  // curr_tm = localtime(&curr_time);
  _pRobot_Status.iMovebase_Result = msgResult->status.status;
  ROS_INFO("[SUCCEEDED]resultCallback: %d ",msgResult->status.status);
}

void BatteryCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  _pRobot_Status.iCallback_Battery = msg->data;
  //printf("_pRobot_Status.iCallback_Battery: %d \n",_pRobot_Status.iCallback_Battery);
}

void ChargingCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  _pRobot_Status.iCallback_Charging_status = msg->data;
  //printf("_pRobot_Status.iCallback_Charging_status: %d \n",_pRobot_Status.iCallback_Charging_status);
}

void EMGCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  _pRobot_Status.iCallback_EMG = msg->data;
}

void BumperCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  _pRobot_Status.iCallback_Bumper = msg->data;
}

//***************************************************************************************************************************************/
//Sevice Function///
bool GotoLocation(string strLocation_name)
{
  bool bResult = false;
  goto_cmd_service->location = strLocation_name;
  while(!goto_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service goto cmd not available, waiting again...");
  }
  auto result = goto_cmd_client->async_send_request(goto_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    _pGoal_pose.dGoal_positionX = result.get()->goal_position_x;
    _pGoal_pose.dGoal_positionY = result.get()->goal_position_y;
    _pGoal_pose.dGoal_quarterX = result.get()->goal_quarter_x;
    _pGoal_pose.dGoal_quarterY = result.get()->goal_quarter_y;
    _pGoal_pose.dGoal_quarterZ = result.get()->goal_quarter_z;
    _pGoal_pose.dGoal_quarterW = result.get()->goal_quarter_w;

    sprintf(Send_buffer,"DS,7,GO1,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
            _pGoal_pose.dGoal_positionX,_pGoal_pose.dGoal_positionY,
            _pGoal_pose.dGoal_quarterX,_pGoal_pose.dGoal_quarterY,_pGoal_pose.dGoal_quarterZ,_pGoal_pose.dGoal_quarterW);

    bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service goto cmd");
    bResult = false;
  }
  return bResult;
}

bool GotoLocation2(double goal_positionX, double goal_positionY, double goal_quarterX, double goal_quarterY, double goal_quarterZ, double goal_quarterW)
{
  bool bResult = false;

  goto_cmd_service2->goal_positionX = goal_positionX;
  goto_cmd_service2->goal_positionY = goal_positionY;
  goto_cmd_service2->goal_quarterX = goal_quarterX;
  goto_cmd_service2->goal_quarterY = goal_quarterY;
  goto_cmd_service2->goal_quarterZ = goal_quarterZ;
  goto_cmd_service2->goal_quarterW = goal_quarterW;

  while(!goto_cmd_client2->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service goto cmd2 not available, waiting again...");
  }
  auto result = goto_cmd_client2->async_send_request(goto_cmd_service2);

  // sprintf(Send_buffer,"DS,7,GO2,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
  //             _pGoal_pose.dGoal_positionX,_pGoal_pose.dGoal_positionY,
  //             _pGoal_pose.dGoal_quarterX,_pGoal_pose.dGoal_quarterY,_pGoal_pose.dGoal_quarterZ,_pGoal_pose.dGoal_quarterW);

  bResult = true;
  return bResult;
}

bool GotoCancel()
{
  bool bResult = false;

  gotocancel_cmd_service->location_id = "";
  while(!gotocancel_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service goto cancel cmd not available, waiting again...");
  }
  auto result = gotocancel_cmd_client->async_send_request(gotocancel_cmd_service);

  bResult = true;
  return bResult;
}

bool Getlocation()
{
  bool bResult = false;
  while(!getlocation_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service get location cmd not available, waiting again...");
  }
  auto result = getlocation_cmd_client->async_send_request(getlocation_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    _pAMCL_pose.dPoseAMCLx = result.get()->pose_amcl_x;
    _pAMCL_pose.dPoseAMCLy = result.get()->pose_amcl_y;

    _pAMCL_pose.dPoseAMCLqx = result.get()->pose_amcl_qx;
    _pAMCL_pose.dPoseAMCLqy = result.get()->pose_amcl_qy;
    _pAMCL_pose.dPoseAMCLqz = result.get()->pose_amcl_qz;
    _pAMCL_pose.dPoseAMCLqw = result.get()->pose_amcl_qw;

    sprintf(Send_buffer, "DS,7,AMCL,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
            _pAMCL_pose.dPoseAMCLx, _pAMCL_pose.dPoseAMCLy,
            _pAMCL_pose.dPoseAMCLqx,_pAMCL_pose.dPoseAMCLqy,_pAMCL_pose.dPoseAMCLqz,_pAMCL_pose.dPoseAMCLqw);

    bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service get location cmd");
    bResult = false;
  }
  return bResult;
}

bool GetDataAll()
{
  bool bResult = false;
  while(!getlocation_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service get data cmd not available, waiting again...");
  }
  auto result = getlocation_cmd_client->async_send_request(getlocation_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    _pAMCL_pose.dPoseAMCLx = result.get()->pose_amcl_x;
    _pAMCL_pose.dPoseAMCLy = result.get()->pose_amcl_y;

    _pAMCL_pose.dPoseAMCLqx = result.get()->pose_amcl_qx;
    _pAMCL_pose.dPoseAMCLqy = result.get()->pose_amcl_qy;
    _pAMCL_pose.dPoseAMCLqz = result.get()->pose_amcl_qz;
    _pAMCL_pose.dPoseAMCLqw = result.get()->pose_amcl_qw;

    sprintf(Send_buffer,"DS,12,DATA,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,XX", 
            _pAMCL_pose.dPoseAMCLx, _pAMCL_pose.dPoseAMCLy,
            _pAMCL_pose.dPoseAMCLqx,_pAMCL_pose.dPoseAMCLqy,_pAMCL_pose.dPoseAMCLqz,_pAMCL_pose.dPoseAMCLqw,
            _pRobot_Status.iCallback_Battery, _pRobot_Status.iCallback_EMG, _pRobot_Status.iCallback_Bumper, _pRobot_Status.iCallback_Charging_status,
            _pRobot_Status.iMovebase_Result);

    bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service get data cmd");
    bResult = false;
  }
  return bResult;
}

bool NavigationMode_ON(string strMap_name)
{
  bool bResult = false;

  navigation_cmd_service->map_name = strMap_name;
  while(!navigation_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service navigation cmd not available, waiting again...");
  }
  auto result = navigation_cmd_client->async_send_request(navigation_cmd_service);

  bResult = true;
  return bResult;
}

bool GetLocation_List()
{
  bool bResult = false;
  string strTemp;

  while(!locationlist_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service get location list cmd not available, waiting again...");
  }
  auto result = locationlist_cmd_client->async_send_request(locationlist_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    m_iTotal_Location_cnt = result.get()->list_num;

    for(int i=0; i<m_iTotal_Location_cnt; i++)
    {
      m_strLocation[i] = result.get()->location_name[i];
      strTemp += (m_strLocation[i] + ",");
    }

    sprintf(Send_buffer,"DS,%d,LOCLIST,%sXX", 
            m_iTotal_Location_cnt+1,
            strTemp.c_str()
            );

    bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service get location list cmd");
    bResult = false;
  }
  return bResult;
}

bool GetMap_List()
{
  bool bResult = false;
  string strTemp;

  while(!maplist_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service get map list cmd not available, waiting again...");
  }
  auto result = maplist_cmd_client->async_send_request(maplist_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    m_iTotal_Map_cnt = result.get()->list_num;

    for(int i=0; i<m_iTotal_Map_cnt; i++)
    {
      m_strMap[i] = result.get()->map_name[i];
      strTemp += (m_strMap[i] + ",");
    }

    sprintf(Send_buffer,"DS,%d,MAPLIST,%sXX", 
            m_iTotal_Map_cnt+1,
            strTemp.c_str()
            );

    bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service get map list cmd");
    bResult = false;
  }
  return bResult;
}

bool Set_Robot_MaxSpeed(double dSpeed)
{
  bool bResult = false;

  setspeed_cmd_service->speed = dSpeed;
  while(!setspeed_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service set robot maxspeed cmd not available, waiting again...");
  }
  auto result = setspeed_cmd_client->async_send_request(setspeed_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    sprintf(Send_buffer,"DS,2,SPEED,%.3f,XX", result.get()->set_vel);

    bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service set robot maxspeed cmd");
    bResult = false;
  }
  return bResult;
}

bool Set_Location(string strLocationName)
{
  bool bResult = false;

  setlocation_cmd_service->location = strLocationName;
  while(!setlocation_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service set location cmd not available, waiting again...");
  }
  auto result = setlocation_cmd_client->async_send_request(setlocation_cmd_service);

  sprintf(Send_buffer,"DS,2,LSV,%s,XX", strLocationName.c_str());

  bResult = true;
  return bResult;
}

bool Set_Output(int Output0, int Output1, int Output2, int Output3, int Output4, int Output5, int Output6, int Output7)
{
  bool bResult = false;

  output_cmd_service->output0 = Output0;
  output_cmd_service->output1 = Output1;
  output_cmd_service->output2 = Output2;
  output_cmd_service->output3 = Output3;
  output_cmd_service->output4 = Output4;
  output_cmd_service->output5 = Output5;
  output_cmd_service->output6 = Output6;
  output_cmd_service->output7 = Output7;
  while(!output_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service set output cmd not available, waiting again...");
  }
  auto result = output_cmd_client->async_send_request(output_cmd_service);

  bResult = true;
  return bResult;
}

//add_230405_wbjin
bool Set_Single_Output(int Output_id, int iValue)
{
  bool bResult = false;
  
  single_output_cmd_service->id = Output_id;
  single_output_cmd_service->value = iValue;
  return bResult;
}

bool Get_GPIO_Status()
{
  bool bResult = false;

  while(!gpio_status_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service get gpio status cmd not available, waiting again...");
  }
  auto result = gpio_status_cmd_client->async_send_request(gpio_status_cmd_service);
  if (rclcpp::spin_until_future_complete(nodes, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
  sprintf(Send_buffer, "DS,17,GPIO,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,XX", 
          result.get()->input0,result.get()->input1,result.get()->input2,
          result.get()->input3,result.get()->input4,result.get()->input5,
          result.get()->input6,result.get()->input7,
          result.get()->output0,result.get()->output1,result.get()->output2,
          result.get()->output3,result.get()->output4,result.get()->output5,
          result.get()->output6,result.get()->output7);

  bResult = true;
  } else {
    RCLCPP_ERROR(nodes->get_logger(), "Failed to call service get gpio status cmd");
    bResult = false;
  }
  return bResult;
}

bool MappingMode_ON()
{
  bool bResult = false;

  while(!mapping_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service mapping mode on cmd not available, waiting again...");
  }
  auto result = mapping_cmd_client->async_send_request(mapping_cmd_service);

  bResult = true;
  return bResult;
}

bool NodeKill()
{
  bool bResult = false;

  while(!nodekill_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service node kill cmd not available, waiting again...");
  }
  auto result = nodekill_cmd_client->async_send_request(nodekill_cmd_service);

  bResult = true;
  return bResult;
}

bool Map_Save(string strMapName)
{
  bool bResult = false;

  mapsave_cmd_service->map_name = strMapName;
  while(!mapsave_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service node kill cmd not available, waiting again...");
  }
  auto result = mapsave_cmd_client->async_send_request(mapsave_cmd_service);

  bResult = true;
  return bResult;
}

bool Docking_Control(int iMarkerID, int iMode)
{
  bool bResult = false;

  //int32 id
  //int32 mode
  dockingcontrol_cmd_service->id = iMarkerID;
  dockingcontrol_cmd_service->mode = iMode;
  while(!dockingcontrol_cmd_client->wait_for_service(1s))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO_STREAM(nodes->get_logger(), "service docking control cmd not available, waiting again...");
  }
  auto result = dockingcontrol_cmd_client->async_send_request(dockingcontrol_cmd_service);

  bResult = true;
  return bResult;
}

//***************************************************************************************************************************************/

void my_handler(sig_atomic_t s)
{
  printf("Caught signal %d\n",s);
  exit(1); 
}

constexpr unsigned int HashCode(const char* str)
{
  return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * HashCode(str + 1) : 8603;
}

bool DoParsing(char* data) 
{
  bool bResult = false;

  char* token;
  char parsing[16][16];
  int iIndex = 0;
  int iCnt = 0;

  token = strtok(data, m_cCOMMA);
  while (token != NULL)
  {
    strcpy(parsing[iIndex++], token);
    token = strtok(NULL, m_cCOMMA);
  }

  //Packet Sorting.../////////////////////////////////////////////////////
  m_cSTX = parsing[0];
  m_cLEN = parsing[1];
  m_cMOD = parsing[2];
  m_cCMD = parsing[3];
  for(int i=0; i < (atoi(m_cLEN)-2); i++)
  {
    m_cPARAM[i] = parsing[4+i];
    iCnt++;
  }
  m_cETX = parsing[4+iCnt];
  
  ///ACK or NAK TEST////////////////////////////////////////
  if(!strcmp(cSTX, m_cSTX) && !strcmp(cETX, m_cETX))
  {
    switch(HashCode(m_cCMD))
    {
      case HashCode("ODOM"): //Odometry data
        if(m_cPARAM[0] == "1") //TETRA origin Odometry
        {
          //printf("m_cPARAM 1 \n");
          sprintf(Send_buffer, "DS,8,ODOM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,XX", 
          _pOdometry.dOdom_position_x, _pOdometry.dOdom_position_y, _pOdometry.dOdom_position_z,
          _pOdometry.dOdom_quaternion_x,_pOdometry.dOdom_quaternion_y,_pOdometry.dOdom_quaternion_z,_pOdometry.dOdom_quaternion_w);
        }
        else // Twist_velocity
        {
        //printf("m_cPARAM 2 \n");
        sprintf(Send_buffer, "DS,3,ODOM,%.3f,%.3f,XX", 
        _pOdometry.dTwist_linear, _pOdometry.dTwist_angular);
        }
        break;
      case HashCode("AMCL"): //Robot Pose data
        Getlocation();
        break;
      case HashCode("STATUS"): //TETRA Status data
        sprintf(Send_buffer, "DS,6,STATUS,%d,%d,%d,%d,%d,XX", 
        _pRobot_Status.iCallback_Battery, _pRobot_Status.iCallback_EMG, _pRobot_Status.iCallback_Bumper, _pRobot_Status.iCallback_Charging_status, _pRobot_Status.iMovebase_Result);
        break;
      case HashCode("MAPLIST"): //Save Map file List data
        GetMap_List();
        break;
      case HashCode("LOCLIST"): //Save WayPoint file List data
        GetLocation_List();
        break;
      case HashCode("NAV"): //Move_base(navigation) Mode Service Call
        NavigationMode_ON(m_cPARAM[0]);
        break;
      case HashCode("SLAM"): //Cartographer Mode Service Call
        MappingMode_ON();
        break;
      case HashCode("MSV"): // Map file Save Service Call
        Map_Save(m_cPARAM[0]);
        break;
      case HashCode("KILL"): //Mapping Mode or Navigation Mode Kill Service Call
        NodeKill();
        break;
      case HashCode("DOC"): // Docking command
        Docking_Control(atoi(m_cPARAM[0]), atoi(m_cPARAM[1]));
        break;
      case HashCode("GO1"): // Move to saved location 
        _pRobot_Status.iMovebase_Result = 0;
        GotoLocation(m_cPARAM[0]);
        break;
      case HashCode("GO2"): // Move to location coordinates
        _pRobot_Status.iMovebase_Result = 0;
        GotoLocation2(atof(m_cPARAM[0]), atof(m_cPARAM[1]), atof(m_cPARAM[2]), atof(m_cPARAM[3]), atof(m_cPARAM[4]), atof(m_cPARAM[5]));
        break;
      case HashCode("GOCXL"): // Move to saved location || Move to location coordinates Cancel
        GotoCancel();
        break;
      case HashCode("SPEED"): // TETRA Navigation Move Speed Set
        Set_Robot_MaxSpeed(atof(m_cPARAM[0]));
        break;
      case HashCode("LSV"): // Save to Location data
        Set_Location(m_cPARAM[0]);
        break;
      case HashCode("OUT"): // GPIO_Output command
        Set_Output(atoi(m_cPARAM[0]),atoi(m_cPARAM[1]),atoi(m_cPARAM[2]),atoi(m_cPARAM[3]),
                  atoi(m_cPARAM[4]),atoi(m_cPARAM[5]),atoi(m_cPARAM[6]),atoi(m_cPARAM[7]));
        break;
      case HashCode("SOUT"): // GPIO_Sigle Output command
        Set_Single_Output(atoi(m_cPARAM[0]),atoi(m_cPARAM[1]));
        break;
      case HashCode("GPIO"): // GPIO Status Check command
        Get_GPIO_Status();
        break;
      case HashCode("DATA"): // AMCL Pose & Robot Status Data all...
        GetDataAll();
        break;
    }
    bResult = true;
    //printf("ACK \n");
    // sprintf(Send_buffer, "ACK");
    write(client_fd, Send_buffer, strlen(Send_buffer));
  }
  else
  {
    bResult = false;
    printf("NAK \n");
    sprintf(Send_buffer, "NAK");
    write(client_fd, Send_buffer, strlen(Send_buffer));
  }    
  return bResult;
}

void *AutoThread_function(void *data)
{
  while(1)
  {
    if(m_bflag_thread)
    {
      msg_size = read(client_fd, buffer, BUF_LEN);
      if(msg_size < 1)
      {
          m_bflag_thread = false;
      }
      else
      {
        printf("[DATA] = %s [msg_size] = %d \n",buffer, msg_size);
        memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer
        DoParsing(buffer);
        memset(&buffer, 0x00, sizeof(buffer)); //clear buffer
      }
    }
    usleep(100000); //10ms
  }
  pthread_cancel(p_auto_thread); //Thread kill
}

void *SocketCheck_Thread_function(void *data)
{
  while(1)
  {
    client_fd = accept(server_fd, (sockaddr *)&client_addr, (socklen_t*)&len);
    if(client_fd < 0)
    {
      printf("Server: accept failed.\n");
      m_bflag_thread = false;
      exit(0);
    }
    inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
    printf("Server : %s client connected.\n", temp);
    m_bflag_thread = true;
  
    usleep(1000000); //100ms
  }
  pthread_cancel(p_auto_thread2); //Thread kill
}

//add _ GUI Button callback fuction...
void TESTCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  if(joy->buttons[5] == 1) //HOME goto...
  {
    _pRobot_Status.iMovebase_Result = 10;
  }
}

int main(int argc, char* argv[])
{
  signal(SIGINT,my_handler);

  rclcpp::init(argc, argv);
  nodes = rclcpp::Node::make_shared("tetra_TCP");

  ////Subscriber//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //ROS msg_subscriber//
  auto odom_sub = nodes->create_subscription<nav_msgs::msg::Odometry>("odom", 100, &OdometryCallback);
  auto result_sub = nodes->create_subscription<move_base_msgs::msg::MoveBaseActionResult>("move_base/result", 10, &resultCallback);

  //Infomation_subscriber//
  auto tetra_battery = nodes->create_subscription<std_msgs::msg::Int32>("tetra_battery", 1, &BatteryCallback);
  auto emg_state = nodes->create_subscription<std_msgs::msg::Int32>("emg_state", 1, &EMGCallback);
  auto bumper_data = nodes->create_subscription<std_msgs::msg::Int32>("bumper_data", 1, &BumperCallback);
  auto docking_status = nodes->create_subscription<std_msgs::msg::Int32>("docking_status", 1, &ChargingCallback);

  //add GUI...
  auto Test_sub = nodes->create_subscription<sensor_msgs::msg::Joy>("/rviz_visual_tools_gui", 10, &TESTCallback);

  ////tetraDS ServiceClient///////////////////////////////////////////////////////////////////////////////////
  goto_cmd_client  = nodes->create_client<tetra_msgs::srv::Gotolocation>("goto_cmd");
  goto_cmd_client2 = nodes->create_client<tetra_msgs::srv::Gotolocation2>("goto_cmd2");
  gotocancel_cmd_client = nodes->create_client<tetra_msgs::srv::Gotocancel>("gotocancel_cmd");
  getlocation_cmd_client = nodes->create_client<tetra_msgs::srv::Getlocation>("getlocation_cmd");
  locationlist_cmd_client = nodes->create_client<tetra_msgs::srv::Getlocationlist>("locationlist_cmd");
  maplist_cmd_client = nodes->create_client<tetra_msgs::srv::Getmaplist>("maplist_cmd");
  setspeed_cmd_client = nodes->create_client<tetra_msgs::srv::Setmaxspeed>("setspeed_cmd");
  setlocation_cmd_client = nodes->create_client<tetra_msgs::srv::Setlocation>("setlocation_cmd");
  navigation_cmd_client = nodes->create_client<tetra_msgs::srv::Runnavigation>("navigation_cmd");
  mapping_cmd_client = nodes->create_client<tetra_msgs::srv::Runmapping>("mapping_cmd");
  mapsave_cmd_client = nodes->create_client<tetra_msgs::srv::Setsavemap>("savemap_cmd");
  nodekill_cmd_client = nodes->create_client<tetra_msgs::srv::Rosnodekill>("nodekill_cmd");
  deletelocation_cmd_client = nodes->create_client<tetra_msgs::srv::Deletelocation>("delete_location_cmd");
  dockingcontrol_cmd_client = nodes->create_client<tetra_msgs::srv::Dockingcontrol>("docking_cmd");
  output_cmd_client = nodes->create_client<tetra_msgs::srv::PowerSetOutport>("Power_outport_cmd");
  gpio_status_cmd_client = nodes->create_client<tetra_msgs::srv::PowerGetIoStatus>("Power_io_status_cmd");
  //add
  single_output_cmd_client = nodes->create_client<tetra_msgs::srv::PowerSetSingleOutport>("Power_single_outport_cmd");

  //***************************************************************************************************************************************/
  //TCP/IP Socket Loop...///
  int port = 5100;
  nodes->declare_parameter("port", 5100);
  port = node->get_parameter("port").as_int();
  if(argc != 2)
  {
    printf("usage : %s [port]\n", argv[0]);
    exit(0);
  }

  if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
  {// 소켓 생성
    printf("Server : Can't open stream socket\n");
    exit(0);
  }
  
  int option = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
  memset(&server_addr, 0x00, sizeof(server_addr));
  //server_Addr 을 NULL로 초기화

  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(port);
  // server_addr.sin_port = htons(atoi(argv[1]));
  //server_addr 셋팅

  if(bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <0)
  {//bind() 호출
    printf("Server : Can't bind local address.\n");
    exit(0);
  }

  if(listen(server_fd, 5) < 0)
  {//소켓을 수동 대기모드로 설정
    printf("Server : Can't listening connect.\n");
    exit(0);
  }

  memset(buffer, 0x00, sizeof(buffer)); //Send_buffer
  printf("Server : wating connection request.\n");
  m_bflag_thread = false;
  len = sizeof(client_addr);

  //***************************************************************************************************************************************/
  /*Thread Create...*/
  int auto_thread_id;
  int a = 1;
  auto_thread_id = pthread_create(&p_auto_thread, NULL, AutoThread_function, (void *)&a);
  if (auto_thread_id < 0)
  {
    printf("auto thread create error !!");
    exit(0);
  }  

  /* SocketCheck Thread Create...*/
  int SocketCheck_thread_id;
  int a2 = 1;
  SocketCheck_thread_id = pthread_create(&p_auto_thread2, NULL, SocketCheck_Thread_function, (void *)&a2);
  if (SocketCheck_thread_id < 0)
  {
    printf("SocketCheck_thread create error !!");
    exit(0);
  } 

  rclcpp::Rate loop_rate(30.0); //default: 30HZ

  while(rclcpp::ok())
  { 
    rclcpp::spin_some(nodes); 
    loop_rate.sleep();
  }

  pthread_cancel(p_auto_thread); //Thread kill
  pthread_cancel(p_auto_thread2); //Thread kill
  close(server_fd);
  
  printf("---server_fd Close--.\n");
  rclcpp::shutdown();
  return 0;
}
