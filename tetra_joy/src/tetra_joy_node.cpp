#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp" 
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BUF_LEN 4096
using namespace std;

serial::Serial ser;
std_msgs::msg::String DataTemp;
std_msgs::msg::String result;
int m_iLinearVelocity = 0;
int m_iAngulerVelocity = 0;


int main (int argc, char** argv)
{
  std::shared_ptr<rclcpp::Node> n = rclcpp::Node::make_shared("tetra_joy_node");

  auto joy_tetra_publisher = n->create_publisher<sensor_msgs::msg::Joy>("joy",rclcpp::SensorDataQoS());
  sensor_msgs::msg::Joy tetra_joy;

  try
  {
    //Bluetooth to USB_serial 
    ser.setPort("/dev/SENA");
    ser.setBaudrate(9600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM(n->get_logger(), "Unable to open port ");
    return -1;
  }

  if(ser.isOpen())
  {
    RCLCPP_INFO_STREAM(n->get_logger(), "Serial Port initialized");
  }
  else
  {
    return -1;
  }

  rclcpp::Time current_time, last_time;
  current_time = n->get_clock()->now();
  last_time = n->get_clock()->now();
  rclcpp::Rate loop_rate(10); //HZ
  
  while(rclcpp::ok())
  {
    rclcpp::spin_some(n);
    current_time = n->get_clock()->now();
    double dt = (current_time - last_time).seconds();

    if(ser.available() > 0)
    {
      DataTemp.data = ser.read();
      if(DataTemp.data == "\n")
      {
        RCLCPP_INFO_STREAM(n->get_logger(), "DATA: " << result.data);
        int ilength = result.data.size();
        int iCommaPoint = result.data.find(','); 
        //ROS_INFO_STREAM("iCommaPoint: " << iCommaPoint);
        if(iCommaPoint < 1) //Check Garbege
        {
          memset(&result.data, 0, sizeof(result.data));
          RCLCPP_INFO_STREAM(n->get_logger(), "Connect Success!");
        }
        else
        {
          m_iLinearVelocity = stof(result.data.substr(0, iCommaPoint));
          m_iAngulerVelocity = stof(result.data.substr(iCommaPoint + 1));
          //ROS_INFO_STREAM("m_iLinearVelocity: " << m_iLinearVelocity);
          //ROS_INFO_STREAM("m_iAngulerVelocity: " << m_iAngulerVelocity);

          //Joy_msg Publish
          //tetra_joy.axes[1] = (float)m_iLinearVelocity;
          tetra_joy.buttons.clear();
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);
          tetra_joy.buttons.push_back(0);

          tetra_joy.axes.clear();
          tetra_joy.axes.push_back(0);
          tetra_joy.axes.push_back((float)m_iLinearVelocity / 100.0);
          tetra_joy.axes.push_back((float)m_iAngulerVelocity / 100.0);
          tetra_joy.axes.push_back(0);
          tetra_joy.axes.push_back(0);
          
          joy_tetra_publisher->publish(tetra_joy);

          memset(&result.data, 0, sizeof(result.data)); 
        }
        
        //usleep(1000);

      }
      else
      {
        result.data += DataTemp.data;
        //ROS_INFO_STREAM("Read: " << DataTemp.data);
      }
        
    }

    //Todo....
    //tetra_joy.axes[0] = m_iLinearVelocity;
    //joy_tetra_publisher.publish(tetra_joy);

    last_time = current_time;
    //ROS_INFO("Time: %.2f\n",dt);
    loop_rate.sleep();
  }
  return 0;
}

