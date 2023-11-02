////TETRA Landmark ROS Package_Ver 0.1
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "message_filters/subscriber.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <sstream>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/types.h>
#include <dirent.h>
#include <error.h>
#include <cstdlib>
#include <algorithm>

//Service
#include "ar_track_alvar_msgs/msg/alvar_markers.hpp" //MSG AR_TAG
//Save Mark ID
#include "tetra_msgs/srv/savemark.hpp" //SRV

#define BUF_LEN 4096
using namespace std;
pthread_t p_thread;

FILE* fp;
int status;
char Textbuffer[BUF_LEN];
double m_baselink2cam_distance = 0.384; //384mm

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

//AR_TAG Pose
int m_iAR_tag_id = -1;
float m_fAR_tag_pose_x = 0.0;
float m_fAR_tag_pose_y = 0.0;
float m_fAR_tag_pose_z = 0.0;
float m_fAR_tag_orientation_x = 0.0;
float m_fAR_tag_orientation_y = 0.0;
float m_fAR_tag_orientation_z = 0.0;
float m_fAR_tag_orientation_w = 0.0;
//Tracked Pose
float m_fTracked_pose_x = 0.0;
float m_fTracked_pose_y = 0.0;
float m_fTracked_pose_z = 0.0;
float m_fTracked_orientation_x = 0.0;
float m_fTracked_orientation_y = 0.0;
float m_fTracked_orientation_z = 0.0;
float m_fTracked_orientation_w = 0.0;
//calc pose
float m_fTracked_calc_pose_x = 0.0;
float m_fTracked_calc_pose_y = 0.0;
//Landmark add..
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_pub;
visualization_msgs::msg::Marker node;
//LandMark Pose//
LANDMARK_POSE _pLandMark;
bool m_bSave_Enable = true;
//**Command srv _ Service Server************************/
tetra_msgs::srv::Savemark savemark_cmd;
rclcpp::Service<tetra_msgs::srv::Savemark>::SharedPtr savemark_service;
//ros Node
std::shared_ptr<rclcpp::Node> nodes;

bool SaveMark_Command(const std::shared_ptr<tetra_msgs::srv::Savemark::Request> req, 
					            std::shared_ptr<tetra_msgs::srv::Savemark::Response> res)
{
	bool bResult = false;
    
  m_bSave_Enable = true;

  /*
  ---
  bool command_Result
  */

	res->command_result = m_bSave_Enable;
	return true;
}

void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  if(joy->buttons[5]) //RB button
	{	
    //Save Landmark Data Enable 
    m_bSave_Enable = true;
	}
}


bool SaveLandMark(LANDMARK_POSE p)
{
  bool bResult = false;

  string m_strFilePathName;
  m_strFilePathName = "/home/tetra/LANDMARK/" + p.ns + ".txt";    
  fp = fopen(m_strFilePathName.c_str(), "w");
  if(fp == NULL)
  { 
    RCLCPP_INFO_ONCE(nodes->get_logger(), "file is null");
    bResult = false;
  }
  else
  {
    if(p.mark_id == m_iAR_tag_id)
    {
      fprintf(fp, "0,%s,%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
              p.header_frame_id.c_str(), 
              p.ns.c_str(), 
              p.mark_id,
              p.pose_position_x,
              p.pose_position_y,
              p.pose_position_z,
              p.pose_position_x + (m_fTracked_pose_x - p.pose_position_x), 
              p.pose_position_y + (m_fTracked_pose_y - p.pose_position_y),
              p.pose_position_z, 
              p.pose_orientation_y - p.pose_orientation_x, 
              p.pose_orientation_w + p.pose_orientation_z
              );

            
      fclose(fp);

      bResult = true;
    }
    else
      bResult = false;
  }

  return bResult;
}

//Map to AR_tag Transform
void Map2Mark_Callback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr req) 
{
  if (!req->markers.empty()) 
  {
    //Green circle LandMark//////////////////////////////////////////
    _pLandMark.header_frame_id = node.header.frame_id = req->markers[0].header.frame_id;
    node.header.stamp = nodes->get_clock()->now();
    node.type = visualization_msgs::msg::Marker::SPHERE;
    _pLandMark.ns = node.ns = "marker_" + std::to_string(req->markers[0].id);
    _pLandMark.mark_id = node.id = req->markers[0].id;

    node.action = visualization_msgs::msg::Marker::ADD; 
    _pLandMark.pose_position_x = node.pose.position.x = req->markers[0].pose.pose.position.x;
    _pLandMark.pose_position_y = node.pose.position.y = req->markers[0].pose.pose.position.y;
    _pLandMark.pose_position_z = node.pose.position.z = req->markers[0].pose.pose.position.z;
    _pLandMark.pose_orientation_x = node.pose.orientation.x = req->markers[0].pose.pose.orientation.x;
    _pLandMark.pose_orientation_y = node.pose.orientation.y = req->markers[0].pose.pose.orientation.y;
    _pLandMark.pose_orientation_z = node.pose.orientation.z = req->markers[0].pose.pose.orientation.z;
    _pLandMark.pose_orientation_w = node.pose.orientation.w = req->markers[0].pose.pose.orientation.w;

    // Points are green 
    node.color.a = 0.8; 
    node.color.r = 0.5;
    node.color.g = 1.0; 
    node.color.b = 0.0;  
    node.scale.x = 0.3;
    node.scale.y = 0.3;
    node.scale.z = 0.3;  

    if(m_bSave_Enable){
      //Publish
      landmark_pub->publish(node);
      //Save Marker data//
      SaveLandMark(_pLandMark);
      m_bSave_Enable = false;
    }
  }
}

//AR_tagCallback
void AR_tagCallback(const ar_track_alvar_msgs::msg::AlvarMarkers::SharedPtr req) 
{
  if (!req->markers.empty()) 
  {
    //AR_Tag data update...
    m_iAR_tag_id = req->markers[0].id;
    m_fAR_tag_pose_x = req->markers[0].pose.pose.position.x;
    m_fAR_tag_pose_y = req->markers[0].pose.pose.position.y;
    m_fAR_tag_pose_z = req->markers[0].pose.pose.position.z;
    m_fAR_tag_orientation_x = req->markers[0].pose.pose.orientation.x;
    m_fAR_tag_orientation_y = req->markers[0].pose.pose.orientation.y;
    m_fAR_tag_orientation_z = req->markers[0].pose.pose.orientation.z;
    m_fAR_tag_orientation_w = req->markers[0].pose.pose.orientation.w;
  }
  else
  {
    m_iAR_tag_id = -1;
  }
}

bool Tracked_pose_Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msgPose)
{
  bool bResult = false;

  m_fTracked_pose_x = msgPose->pose.position.x;
  m_fTracked_pose_y = msgPose->pose.position.y;
  m_fTracked_pose_z = msgPose->pose.position.z;
  m_fTracked_orientation_x = msgPose->pose.orientation.x;
  m_fTracked_orientation_y = msgPose->pose.orientation.y;
  m_fTracked_orientation_z = msgPose->pose.orientation.z;
  m_fTracked_orientation_w = msgPose->pose.orientation.w;

  bResult = true;
  return bResult;
}


int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nodes = rclcpp::Node::make_shared("tetra_landmark_save");
  //AR_TAG_subscriber//
  rclcpp::Subscription<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr map2marker_sub = nodes->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>("map_to_marker_pose", rclcpp::SensorDataQoS(), &Map2Mark_Callback);
  //AR_TAG_subscriber//
  rclcpp::Subscription<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr AR_sub = nodes->create_subscription<ar_track_alvar_msgs::msg::AlvarMarkers>("ar_pose_marker", rclcpp::SensorDataQoS(), &AR_tagCallback);
  //tracked_pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_sub = nodes->create_subscription<geometry_msgs::msg::PoseStamped>("tracked_pose", rclcpp::SystemDefaultsQoS(), &Tracked_pose_Callback);

  //landmark add..
  landmark_pub = nodes->create_publisher<visualization_msgs::msg::Marker>("marker/node", rclcpp::SensorDataQoS());
  
  //Joystick add...
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub = nodes->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), &joyCallback);
  
  rclcpp::Rate loop_rate(30.0);  //30hz

  //Command Service//
  savemark_service = nodes->create_service<tetra_msgs::srv::Savemark>("savemark_cmd", &SaveMark_Command);

  while(rclcpp::ok())
  {
    rclcpp::spin_some(nodes);
  
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
