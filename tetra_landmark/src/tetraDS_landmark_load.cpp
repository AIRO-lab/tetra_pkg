////TETRA_DS Landmark ROS Package_Ver 0.1
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

#define BUF_LEN 4096
using namespace std;
pthread_t p_thread;

FILE* fp;
int status;
char Textbuffer[BUF_LEN];

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

//Landmark add..
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_pub;
visualization_msgs::msg::Marker node;
//LandMark Pose//
LANDMARK_POSE _pLandMark[255];
//Landmark info//
int m_iTotal_landmark_num = 0;
//ros Node
std::shared_ptr<rclcpp::Node> nodes;

bool OpenLocationFile(int ifile_cnt, string str_location)
{
  bool bResult = false;
  string m_strFilePathName;
  string m_header_frame_id;
  string m_ns;
  m_strFilePathName = "/home/tetra/LANDMARK/" + str_location;  
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
            case 0: //header_frame_id.c_str()
              if(ptr != NULL)
              {
                _pLandMark[ifile_cnt].header_frame_id = ptr;
                printf("header_frame_id: %s \n", _pLandMark[ifile_cnt].header_frame_id.c_str());  
              }
              break;
            case 1: //ns.c_str()
              _pLandMark[ifile_cnt].ns = ptr;
              printf("ns: %s \n", _pLandMark[ifile_cnt].ns.c_str());
              break;
            case 2: //mark_id
              _pLandMark[ifile_cnt].mark_id = atoi(ptr);
              printf("mark_id: %d \n", _pLandMark[ifile_cnt].mark_id);
              break;
            case 3: //pose_position_x
              _pLandMark[ifile_cnt].pose_position_x = atof(ptr);
              printf("pose_position_x: %f \n", _pLandMark[ifile_cnt].pose_position_x);
              break;
            case 4: //pose_position_y
              _pLandMark[ifile_cnt].pose_position_y = atof(ptr);
              printf("pose_position_y: %f \n", _pLandMark[ifile_cnt].pose_position_y);
              break;
            case 5: //pose_position_z
              _pLandMark[ifile_cnt].pose_position_z = atof(ptr);
              printf("pose_position_z: %f \n", _pLandMark[ifile_cnt].pose_position_z);
              break;
          }
          icnt++;
        }

        printf("************************** \n");
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
    RCLCPP_INFO_ONCE(nodes->get_logger(), "File Open Fail: %s", str_location.c_str());
    bResult = false;
  }
  return bResult;
}

int LocationList_Command()
{
  int iResult = -1;

  //Load File List
  int m_icnt =0;
  DIR *dir;
  struct dirent *ent;
  dir = opendir("/home/tetra/LANDMARK/"); // Landmark file path
  if (dir != NULL) 
  {
    //print all the files and directories within directory
    while ((ent = readdir(dir)) != NULL)
    {
      if(strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) 
      { 
        continue; 
      }
      OpenLocationFile(m_icnt, ent->d_name);
      m_icnt++;
    }

    //Total List number
    iResult = m_icnt;
    closedir (dir);
  } 
  else 
  {
    //could not open directory
    perror ("");
    iResult = -1;
  }
  return iResult;
}

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nodes = rclcpp::Node::make_shared("tetra_landmark_load");

  //landmark add..
  landmark_pub = nodes->create_publisher<visualization_msgs::msg::Marker>("marker/node", 100);

  m_iTotal_landmark_num = LocationList_Command();
  printf("m_iTotal_landmark_num: %d \n", m_iTotal_landmark_num);

  rclcpp::Rate loop_rate(1.0);  //1Hz

  while(rclcpp::ok())
  {
    rclcpp::spin_some(nodes);
    //Draw Landmark//
    for(int i=0; i<m_iTotal_landmark_num; i++)
    {
      node.header.frame_id = _pLandMark[i].header_frame_id;
      node.header.stamp = nodes->get_clock()->now();
      node.type = visualization_msgs::msg::Marker::SPHERE;
      node.ns = _pLandMark[i].ns;
      node.id = _pLandMark[i].mark_id;
      node.action = visualization_msgs::msg::Marker::ADD; 
      node.pose.position.x = _pLandMark[i].pose_position_x;
      node.pose.position.y = _pLandMark[i].pose_position_y;
      node.pose.position.z = _pLandMark[i].pose_position_z;
      
      node.pose.orientation.x = 0.0;
      node.pose.orientation.y = 0.0; 
      node.pose.orientation.z = 0.0; 
      node.pose.orientation.w = 1.0; 
      /*
      node.pose.orientation.x = _pLandMark[i].pose_orientation_x;
      node.pose.orientation.y = _pLandMark[i].pose_orientation_y; 
      node.pose.orientation.z = _pLandMark[i].pose_orientation_z; 
      node.pose.orientation.w = _pLandMark[i].pose_orientation_w;
      */ 
      // Points are green 
      node.color.a = 0.8; 
      node.color.r = 0.5;
      node.color.g = 1.0;
      node.color.b = 0.0;  
      node.scale.x = 0.3;
      node.scale.y = 0.3;
      node.scale.z = 0.3;
      //Publish
      landmark_pub->publish(node);
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
