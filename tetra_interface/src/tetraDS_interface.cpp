#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp" //Ultrasonic//
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tetra_msgs/srv/ledcontrol.hpp" //SRV
#include "tetra_msgs/srv/ledtogglecontrol.hpp" //SRV
#include "tetra_msgs/srv/toggleon.hpp" //SRV
#include "tetra_msgs/srv/integrallog.hpp" //SRV
#include "tetra_msgs/srv/power_set_outport.hpp" //SRV 
#include "tetra_msgs/msg/gpio.hpp" //MSG
#include "tetra_msgs/srv/loadcell_callibration.hpp" //SRV
#include "tetra_msgs/srv/conveyor_auto_movement.hpp" //SRV
#include "tetra_msgs/srv/conveyor_manual_movement.hpp" //SRV
#include "tetra_msgs/srv/power_get_io_status.hpp" //SRV

#include "tetra_msgs/srv/power_set_enable.hpp" //SRV
#include "tetra_msgs/srv/power_set_single_outport.hpp" //SRV
#include "tetra_msgs/srv/power_set_single_enable.hpp" //SRV
#include "tetra_msgs/srv/power_wheel_enable.hpp" //SRV
#include "tetra_msgs/srv/power_parameter_read.hpp" // SRV
#include "tetra_msgs/srv/conveyor_parameter_read.hpp" // SRV
#include "tetra_msgs/srv/power_parameter_write.hpp" // SRV
#include "tetra_msgs/srv/conveyor_parameter_write.hpp" // SRV
#include "tetra_msgs/srv/conveyor_data_read.hpp" // SRV
#include "tetra_msgs/srv/power_data_read.hpp" // SRV
#include "tetra_msgs/srv/power_version_read.hpp" // SRV
#include "tetra_msgs/srv/power_sonar_read.hpp" // SRV
#include "tetra_msgs/srv/power_adc_read.hpp" // SRV
#include "tetra_msgs/srv/power_sonar_cmd.hpp" //SRV



extern "C"
{
	#include "power_module.h"
	#include "dssp_rs232_power_module.h"
}

#include <thread>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
using namespace std;

#define Ultrasonic_MIN_range	0.04
#define Ultrasonic_MAX_range	0.5
#define BUF_LEN 4096

//Power parameter data
int m_iParam0 = 0;
int m_iParam1 = 0;
int m_iParam2 = 0;
int m_iParam3 = 0;
int m_iParam4 = 0;
int m_iParam5 = 0;
int m_iParam6 = 0;
int m_iParam7 = 0;
int m_iParam8 = 0;
int m_iParam9 = 0;
int m_iParam10 = 0;
int m_iParam11 = 0;
int m_iParam12 = 0;
int m_iParam13 = 0;
int m_iParam14 = 0;
int m_iParam15 = 0;
int m_iParam16 = 0;
int m_iParam17 = 0;
int m_iParam18 = 0;
int m_iParam19 = 0;
int m_iParam20 = 0;
int m_iParam21 = 0;
int m_iParam22 = 0;
int m_iParam23 = 0;
int m_iParam24 = 0;
int m_iParam25 = 0;
int m_iParam26 = 0;
int m_iParam27 = 0;
int m_iParam28 = 0;
int m_iParam29 = 0;
int m_iParam30 = 0;
int m_iParam31 = 0;

int m_iFlag_PowerCheck_cnt = 0;
int com_port = 0;
char port[16] = {0,};
//battery data
int m_ibattery_Level = 0;
double m_dbattery = 0.0;
double m_dVoltage = 0.0;
double m_dCurrent = 0.0;
int m_imode_status = 0;
int m_iPowerCheck = 0;
int m_iPowerCheckCount = 0;
//Ultrasonic data//
double m_dUltrasonic[8] = {0.0, };  //Max Ultrasonic: 8ea (TETRA-DS5 used 4ea)
sensor_msgs::msg::Range range_msg1; //Ultrasonic_1
sensor_msgs::msg::Range range_msg2; //Ultrasonic_2
sensor_msgs::msg::Range range_msg3; //Ultrasonic_3
sensor_msgs::msg::Range range_msg4; //Ultrasonic_4
double time_offset_in_seconds;
//GPIO data//
int m_iOutput[8] = {0,};
int m_iInput[8] = {0,};
//Conveyor loadcell & sensor status//
double m_dLoadcell_weight = 0.0;
int m_dConveyor_sensor = 0; 
int m_iConveyor_movement = 0;
bool m_bConveyor_option = true;

std::shared_ptr<rclcpp::Node> node;

//ROS tetra_msgs custom service
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr chargeport_service_on;
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr chargeport_service_off;
rclcpp::Service<tetra_msgs::srv::Ledcontrol>::SharedPtr led_service;
tetra_msgs::srv::Ledcontrol led_cmd;
rclcpp::Service<tetra_msgs::srv::Ledtogglecontrol>::SharedPtr ledtoggle_service;
tetra_msgs::srv::Ledtogglecontrol ledtoggle_cmd;
rclcpp::Service<tetra_msgs::srv::Toggleon>::SharedPtr turnon_service;
tetra_msgs::srv::Toggleon turnon_cmd;
rclcpp::Service<tetra_msgs::srv::Integrallog>::SharedPtr log_service;
tetra_msgs::srv::Integrallog log_cmd;
//GPIO_service
rclcpp::Service<tetra_msgs::srv::PowerSetOutport>::SharedPtr power_outport_service;
tetra_msgs::srv::PowerSetOutport Power_outport_cmd;

rclcpp::Service<tetra_msgs::srv::PowerSetSingleOutport>::SharedPtr power_single_outport_service;
tetra_msgs::srv::PowerSetSingleOutport Power_single_outport_cmd;

rclcpp::Service<tetra_msgs::srv::PowerGetIoStatus>::SharedPtr power_get_io_service;
tetra_msgs::srv::PowerGetIoStatus Power_io_status_cmd;

//GPIO msg
tetra_msgs::msg::Gpio gpio_msg;
rclcpp::Publisher<tetra_msgs::msg::Gpio>::SharedPtr GPIO_pub;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_1;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_2;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_3;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_4;

//Conveyor Loadcell CAL
rclcpp::Service<tetra_msgs::srv::LoadcellCallibration>::SharedPtr loadcell_callibration_service;
tetra_msgs::srv::LoadcellCallibration CAL_cmd;
//Conveyor Auto Movement
rclcpp::Service<tetra_msgs::srv::ConveyorAutoMovement>::SharedPtr conveyor_auto_movement_service;
tetra_msgs::srv::ConveyorAutoMovement Auto_Move_cmd;
//Conveyor Manual Movement
rclcpp::Service<tetra_msgs::srv::ConveyorManualMovement>::SharedPtr conveyor_manual_movement_service;
tetra_msgs::srv::ConveyorManualMovement Manual_Move_cmd;

//POWER Enalbe service
rclcpp::Service<tetra_msgs::srv::PowerSetEnable>::SharedPtr power_enable_service;
tetra_msgs::srv::PowerSetEnable Power_enable_cmd; 
rclcpp::Service<tetra_msgs::srv::PowerSetSingleEnable>::SharedPtr power_single_enable_service;
tetra_msgs::srv::PowerSetSingleEnable Power_single_enable_cmd;
rclcpp::Service<tetra_msgs::srv::PowerWheelEnable>::SharedPtr power_wheel_enable_service;
tetra_msgs::srv::PowerWheelEnable Power_wheel_enable_cmd;
//PowerSensor SONAR START
rclcpp::Service<tetra_msgs::srv::PowerSonarCmd>::SharedPtr power_sonar_cmd_service;
tetra_msgs::srv::PowerSonarCmd Power_sonar_start_cmd;

//tetra parameter read & write service
rclcpp::Service<tetra_msgs::srv::PowerParameterRead>::SharedPtr power_parameter_read_service;
tetra_msgs::srv::PowerParameterRead Power_parameter_read_cmd;
rclcpp::Service<tetra_msgs::srv::ConveyorParameterRead>::SharedPtr conveyor_parameter_read_service;
tetra_msgs::srv::ConveyorParameterRead Conveyor_parameter_read_cmd;
rclcpp::Service<tetra_msgs::srv::PowerParameterWrite>::SharedPtr power_parameter_write_service;
tetra_msgs::srv::PowerParameterWrite Power_parameter_write_cmd;
rclcpp::Service<tetra_msgs::srv::ConveyorParameterWrite>::SharedPtr conveyor_parameter_write_service;
tetra_msgs::srv::ConveyorParameterWrite Conveyor_parameter_write_cmd;

//tetra data read service
rclcpp::Service<tetra_msgs::srv::ConveyorDataRead>::SharedPtr conveyor_data_read_service;
tetra_msgs::srv::ConveyorDataRead Conveyor_data_read_cmd;
rclcpp::Service<tetra_msgs::srv::PowerDataRead>::SharedPtr power_data_read_service;
tetra_msgs::srv::PowerDataRead Power_data_read_cmd;
rclcpp::Service<tetra_msgs::srv::PowerVersionRead>::SharedPtr power_version_read_service;
tetra_msgs::srv::PowerVersionRead Power_version_read_cmd;
rclcpp::Service<tetra_msgs::srv::PowerSonarRead>::SharedPtr power_sonar_read_service;
tetra_msgs::srv::PowerSonarRead Power_sonar_read_cmd;
rclcpp::Service<tetra_msgs::srv::PowerAdcRead>::SharedPtr power_adc_read_service;
tetra_msgs::srv::PowerAdcRead Power_adc_read_cmd;


//File read & write
FILE *fp;
int status;
char Textbuffer[BUF_LEN];
char strPath[BUF_LEN];

bool Log_Command(const std::shared_ptr<tetra_msgs::srv::Integrallog::Request> req, 
		std::shared_ptr<tetra_msgs::srv::Integrallog::Response> res)
{
	bool bResult = false;

	time_t timer; 
	struct tm* t; 
	timer = time(NULL); 
	t = localtime(&timer); 

	int m_iVlotage_data[500] = {0, };
	int m_iCurrent_data[500] = {0, };
	string m_strFilePathName;
	string m_colon = ":";
	string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
	string m_temp1 = m_time_stemp + "_V_log";
	string m_temp2 = m_time_stemp + "_I_log";
    
	if(req->value_index == 1)
	{
		m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
		dssp_rs232_power_module_read_Voltage(m_iVlotage_data); 

		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL) 
		  RCLCPP_INFO_ONCE(node->get_logger(), "file is null");

		for(int i=0; i<500; i++)
		{
			fprintf(fp, "%.2f \n", (double)m_iVlotage_data[i] / 10.0);
		}
		fclose(fp);
	}
	else if(req->value_index == 2)
	{
		m_strFilePathName = "/home/tetra/LOG/" + m_temp2 + ".txt";  
		dssp_rs232_power_module_read_Current(m_iCurrent_data);

		fp = fopen(m_strFilePathName.c_str(), "w");
		if(fp == NULL) 
			RCLCPP_INFO_ONCE(node->get_logger(), "file is null");

		for(int j=0; j<500; j++)
		{
			fprintf(fp, "%.2f \n", (double)m_iCurrent_data[j] / 10.0);
		}
		fclose(fp);
	}
  /*
	int32 value_index
	---
	bool command_result
  */
  bResult = true;
	res->command_result = bResult;
	return true;
}

void Error_Log_write(string log_data)
{
	time_t timer; 
	struct tm* t; 
	timer = time(NULL); 
	t = localtime(&timer); 

	string m_strFilePathName;
	string m_colon = ":";
	string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
	string m_temp1 = m_time_stemp + "_error";		

	m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
	fp = fopen(m_strFilePathName.c_str(), "w");
	if(fp == NULL) 
		RCLCPP_INFO_ONCE(node->get_logger(), "file is null");

	fprintf(fp, "[Error]: %s \n", log_data.c_str());
	fclose(fp);
}

bool LEDcontrol_Command(const std::shared_ptr<tetra_msgs::srv::Ledcontrol::Request> req, 
			std::shared_ptr<tetra_msgs::srv::Ledcontrol::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_light(req->id, req->led_brightness);
  /*
  int32 id
	int32 led_brightness
  ---
  bool command_Result
  */
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool LEDtoggle_Command(const std::shared_ptr<tetra_msgs::srv::Ledtogglecontrol::Request> req, 
			std::shared_ptr<tetra_msgs::srv::Ledtogglecontrol::Response> res)
{
	bool bResult = false;

	dssp_rs232_power_module_set_light_toggle(req->de_index, req->light_accel, 
						req->led_high_brightness, req->light_decel, req->led_low_brightness);
													 
	/*
	int32 de_index
  int32 light_accel
  int32 led_high_brightness
  int32 light_decel
  int32 led_low_brightness
  ---
  bool command_result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool TurnOn_Command(const std::shared_ptr<tetra_msgs::srv::Toggleon::Request> req, 
			std::shared_ptr<tetra_msgs::srv::Toggleon::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_toggle_on(req->id);
  /*
  int32 id
  ---
  bool command_result
  */
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool ChargingPortOn(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
	dssp_rs232_power_module_set_charging_ready(1);
	return true;
}

bool ChargingPortOff(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
	dssp_rs232_power_module_set_charging_ready(0);
	return true;
}

bool OutportOnOff(const std::shared_ptr<tetra_msgs::srv::PowerSetOutport::Request> req, 
			std::shared_ptr<tetra_msgs::srv::PowerSetOutport::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_OutputPort(req->output0,req->output1,req->output2,req->output3,req->output4,req->output5,req->output6,req->output7);
  /*
  int8 output0
	int8 output1
	int8 output2
	int8 output3
	int8 output4
	int8 output5
	int8 output6
	int8 output7
	---
	bool command_result
  */
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool SingleOutportOnOff(const std::shared_ptr<tetra_msgs::srv::PowerSetSingleOutport::Request> req, 
			std::shared_ptr<tetra_msgs::srv::PowerSetSingleOutport::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_Single_OutputPort(req->id,req->value);
   
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_get_IO_Status(const std::shared_ptr<tetra_msgs::srv::PowerGetIoStatus::Request> req, 
			 std::shared_ptr<tetra_msgs::srv::PowerGetIoStatus::Response> res)
{
	bool bResult = false;
    
	//Input data
	res->input0 = gpio_msg.input0;
	res->input1 = gpio_msg.input1;
	res->input2 = gpio_msg.input2;
	res->input3 = gpio_msg.input3;
	res->input4 = gpio_msg.input4;
	res->input5 = gpio_msg.input5;
	res->input6 = gpio_msg.input6;
	res->input7 = gpio_msg.input7;
	//Output data
	res->output0 = gpio_msg.output0;
	res->output1 = gpio_msg.output1;
	res->output2 = gpio_msg.output2;
	res->output3 = gpio_msg.output3;
	res->output4 = gpio_msg.output4;
	res->output5 = gpio_msg.output5;
	res->output6 = gpio_msg.output6;
	res->output7 = gpio_msg.output7;
	
  /*
	uint8  input0
	uint8  input1
	uint8  input2
	uint8  input3
	uint8  input4
	uint8  input5
	uint8  input6
	uint8  input7
	uint8  output0
	uint8  output1
	uint8  output2
	uint8  output3
	uint8  output4
	uint8  output5
	uint8  output6
	uint8  output7
	bool   command_result
  */

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Loadcell_Callibration_Command(const std::shared_ptr<tetra_msgs::srv::LoadcellCallibration::Request> req, 
					std::shared_ptr<tetra_msgs::srv::LoadcellCallibration::Response> res)
{
	bool bResult = false;

	dssp_rs232_power_module_loadcell_callibration();
													 
	/*
  ---
  bool command_Result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Conveyor_Auto_Move_Command(const std::shared_ptr<tetra_msgs::srv::ConveyorAutoMovement::Request> req, 
					std::shared_ptr<tetra_msgs::srv::ConveyorAutoMovement::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_conveyor_movement(req->start);
  /*
  int32 start
  ---
  bool command_Result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Conveyor_Manual_Move_Command(const std::shared_ptr<tetra_msgs::srv::ConveyorManualMovement::Request> req, 
					std::shared_ptr<tetra_msgs::srv::ConveyorManualMovement::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_conveyor_manual_movement(req->mode);
  /*
  int32 mode
  ---
  bool command_Result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

void RangeToCloud_D_L(const sensor_msgs::msg::Range::SharedPtr range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		pcl::PointXYZ pcl_point;
		pcl_point.x = range_msg->range;
		pcl_point.y = 0.0;
		pcl_point.z = 0.0;
		cloud->points.push_back(pcl_point);
		++cloud->width;
		sensor_msgs::msg::PointCloud2 cloudmsg;
		pcl::toROSMsg(*cloud, cloudmsg);
		cloudmsg.header.frame_id = "sonar_DL";
		points_1->publish(cloudmsg);
	}
}

void RangeToCloud_R_L(const sensor_msgs::msg::Range::SharedPtr range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		pcl::PointXYZ pcl_point;
		pcl_point.x = range_msg->range;
		pcl_point.y = 0.0;
		pcl_point.z = 0.0;
		cloud->points.push_back(pcl_point);
		++cloud->width;
		sensor_msgs::msg::PointCloud2 cloudmsg;
		pcl::toROSMsg(*cloud, cloudmsg);
		cloudmsg.header.frame_id = "sonar_DL";
		points_2->publish(cloudmsg);
	}
}

void RangeToCloud_R_R(const sensor_msgs::msg::Range::SharedPtr range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		pcl::PointXYZ pcl_point;
		pcl_point.x = range_msg->range;
		pcl_point.y = 0.0;
		pcl_point.z = 0.0;
		cloud->points.push_back(pcl_point);
		++cloud->width;
		sensor_msgs::msg::PointCloud2 cloudmsg;
		pcl::toROSMsg(*cloud, cloudmsg);
		cloudmsg.header.frame_id = "sonar_DL";
		points_3->publish(cloudmsg);
	}
}

void RangeToCloud_D_R(const sensor_msgs::msg::Range::SharedPtr range_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->header.stamp = pcl_conversions::toPCL(range_msg->header).stamp;
	
	cloud->height = 1;

	if (range_msg->range < std::numeric_limits<float>::infinity())
	{
		pcl::PointXYZ pcl_point;
		pcl_point.x = range_msg->range;
		pcl_point.y = 0.0;
		pcl_point.z = 0.0;
		cloud->points.push_back(pcl_point);
		++cloud->width;
		sensor_msgs::msg::PointCloud2 cloudmsg;
		pcl::toROSMsg(*cloud, cloudmsg);
		cloudmsg.header.frame_id = "sonar_DL";
		points_4->publish(cloudmsg);
	}
}

bool PowerEnableOnOff(const std::shared_ptr<tetra_msgs::srv::PowerSetEnable::Request> req, 
			std::shared_ptr<tetra_msgs::srv::PowerSetEnable::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_Enable(req->power0,req->power1,req->power2,req->power3,req->power4,req->power5,req->power6,req->power7);
	
  /*
  int8 power0
	int8 power1
	int8 power2
	int8 power3
	int8 power4
	int8 power5
	int8 power6
	int8 power7
	---
	bool command_Result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool PowerSingleEnableOnOff(const std::shared_ptr<tetra_msgs::srv::PowerSetSingleEnable::Request> req, 
				 std::shared_ptr<tetra_msgs::srv::PowerSetSingleEnable::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_set_Single_Enable(req->id,req->value);
	   
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool PowerWheelEnableOnOff(const std::shared_ptr<tetra_msgs::srv::PowerWheelEnable::Request> req, 
				std::shared_ptr<tetra_msgs::srv::PowerWheelEnable::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_wheel_enable(req->on);
  /*
  int32 start
  ---
  bool command_Result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_parameter_Read_Command(const std::shared_ptr<tetra_msgs::srv::PowerParameterRead::Request> req, 
					std::shared_ptr<tetra_msgs::srv::PowerParameterRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_parameter_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20,&m_iParam21,&m_iParam22,&m_iParam23,&m_iParam24,&m_iParam25,&m_iParam26,&m_iParam27,&m_iParam28,&m_iParam29,&m_iParam30,&m_iParam31);
	res->p0_uart1_baudrate = m_iParam0;
	res->p1_uart2_baudrate = m_iParam1;
	res->p2_sonar_select = m_iParam2;
	res->p3_sonar_distance_offset = m_iParam3;
	res->p4_sonar_quantity = m_iParam4;
	res->p5_sonar_max_distance = m_iParam5;
	res->p6_nc = m_iParam6;
	res->p7_sonar_none_detect_mode = m_iParam7;
	res->p8_battery_charging_offset = m_iParam8;
	res->p9_battery_charging_gain = m_iParam9;
	res->p10_status_led_min = m_iParam10;
	res->p11_status_led_max = m_iParam11;
	res->p12_battery_led_min = m_iParam12;
	res->p13_battery_led_max = m_iParam13;
	res->p14_led_ch1_offset = m_iParam14;
	res->p15_led_ch2_offset = m_iParam15;
	res->p16_led_ch3_offset = m_iParam16;
	res->p17_led_ch4_offset = m_iParam17;
	res->p18_led_ch5_offset = m_iParam18;
	res->p19_led_ch6_offset = m_iParam19;
	res->p20_led_ch7_offset = m_iParam20;
	res->p21_led_ch8_offset = m_iParam21;
	res->p22_power_enable = m_iParam22;
	res->p23_outport = m_iParam23;
	res->p24_battery_recharge_voltage = m_iParam24;
	res->p25_battery_recharge_offset = m_iParam25;
	res->p26_battery_min = m_iParam26;
	res->p27_battery_max = m_iParam27;
	res->p28_battery_sampling_time = m_iParam28;
	res->p29_conveyor_mode = m_iParam29;
	res->p30_conveyor_terminal_base = m_iParam30;
	res->p31_conveyor_terminal_offset = m_iParam31;
	/*
	int32 num
	---
	int32 data
	bool command_Result
    */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Conveyor_parameter_Read_Command(const std::shared_ptr<tetra_msgs::srv::ConveyorParameterRead::Request> req, 
					std::shared_ptr<tetra_msgs::srv::ConveyorParameterRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_conveyor_module_parameter_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20);
	res->p0_uart1_baudrate = m_iParam0;
	res->p1_uart2_baudrate = m_iParam1;
	res->p2_mode_select = m_iParam2;
	res->p3_outport = m_iParam3;
	res->p4_loadcell_zero = m_iParam4;
	res->p5_loadcell_min = m_iParam5;
	res->p6_loadcell_gain = m_iParam6;
	res->p7_conveyor_dir = m_iParam7;
	res->p8_conveyor_timeout = m_iParam8;
	res->p9_fix_loading_start_delay = m_iParam9;
	res->p10_fix_loading_end_delay = m_iParam10;
	res->p11_fix_unloading_start_delay = m_iParam11;
	res->p12_fix_unloading_motor_delay = m_iParam12;
	res->p13_fix_unloading_end_delay = m_iParam13;
	res->p14_loading_start_delay = m_iParam14;
	res->p15_loading_end_delay = m_iParam15;
	res->p16_unloading_start_delay = m_iParam16;
	res->p17_unloading_end_delay = m_iParam17;
	res->p18_express_quantity = m_iParam18;
	res->p19_terminal_loading_base = m_iParam19;
	res->p20_terminal_unloading_base = m_iParam20;
	
	/*
	int32 num
	---
	int32 data
	bool command_Result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Conveyor_data_Read_Command(const std::shared_ptr<tetra_msgs::srv::ConveyorDataRead::Request> req, 
				std::shared_ptr<tetra_msgs::srv::ConveyorDataRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_conveyor_module_data_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13);
	res->d0_docking_signal0 = m_iParam0;
	res->d1_docking_signal1 = m_iParam1;
	res->d2_charge_current = m_iParam2;
	res->d3_charge_signal = m_iParam3;
	res->d4_charge_voltage = m_iParam4;
	res->d5_loadcell_low = m_iParam5;
	res->d6_loadcell_sen = m_iParam6;
	res->d7_ad_in1 = m_iParam7;
	res->d8_ad_in2 = m_iParam8;
	res->d9_ad_in3 = m_iParam9;
	res->d10_in_status = m_iParam10;
	res->d11_out_status = m_iParam11;
	res->d12_photo_status = m_iParam12;
	res->d13_conveyor_status = m_iParam13;
		
	/*
	int32 num
	---
	int32 data
	bool command_Result
  */
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_data_Read_Command(const std::shared_ptr<tetra_msgs::srv::PowerDataRead::Request> req, 
				std::shared_ptr<tetra_msgs::srv::PowerDataRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_data_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11);
	res->d0_battery_voltage = m_iParam0;
	res->d1_system_current = m_iParam1;
	res->d2_charge_current = m_iParam2;
	res->d3_charge_signal = m_iParam3;
	res->d4_inport_status = m_iParam4;
	res->d5_outport_status = m_iParam5;
	res->d6_power_status = m_iParam6;
	res->d7_charger_terminal_status = m_iParam7;
	res->d8_temperature0 = m_iParam8;
	res->d9_temperature1 = m_iParam9;
	res->d10_mobd_inport_status = m_iParam10;
	res->d11_mobd_outport_status = m_iParam11;		
	
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_version_Read_Command(const std::shared_ptr<tetra_msgs::srv::PowerVersionRead::Request> req, 
				std::shared_ptr<tetra_msgs::srv::PowerVersionRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_version_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);
	res->d0_board_type = m_iParam0;
	res->d1_year2 = m_iParam1;
	res->d2_year1 = m_iParam2;
	res->d3_month2 = m_iParam3;
	res->d4_month1 = m_iParam4;
	res->d5_day2 = m_iParam5;
	res->d6_day1 = m_iParam6;
	res->d7_ver = m_iParam7;

  bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_sonar_Read_Command(const std::shared_ptr<tetra_msgs::srv::PowerSonarRead::Request> req, 
				std::shared_ptr<tetra_msgs::srv::PowerSonarRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_sonar_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5);
	res->d0_sonar0 = m_iParam0;
	res->d1_sonar1 = m_iParam1;
	res->d2_sonar2 = m_iParam2;
	res->d3_sonar3 = m_iParam3;
	res->d4_sonar4 = m_iParam4;
	res->d5_sonar5 = m_iParam5;
	
  bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_adc_Read_Command(const std::shared_ptr<tetra_msgs::srv::PowerAdcRead::Request> req, 
				std::shared_ptr<tetra_msgs::srv::PowerAdcRead::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_adc_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);
	res->d0_adc0 = m_iParam0;
	res->d1_adc1 = m_iParam1;
	res->d2_adc2 = m_iParam2;
	res->d3_adc3 = m_iParam3;
	res->d4_adc4 = m_iParam4;
	res->d5_adc5 = m_iParam5;
	res->d6_adc6 = m_iParam6;
	res->d7_adc7 = m_iParam7;
				
	/*
	int32 num
	---
	int32 data
	bool command_result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_parameter_Write_Command(const std::shared_ptr<tetra_msgs::srv::PowerParameterWrite::Request> req, 
					std::shared_ptr<tetra_msgs::srv::PowerParameterWrite::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_power_module_parameter_write(req->num, req->data);
	/*
	int32 num
	int32 data
	---
	bool command_result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Conveyor_parameter_Write_Command(const std::shared_ptr<tetra_msgs::srv::ConveyorParameterWrite::Request> req, 
					std::shared_ptr<tetra_msgs::srv::ConveyorParameterWrite::Response> res)
{
	bool bResult = false;
    
	dssp_rs232_conveyor_module_parameter_write(req->num, req->data);
	/*
	int32 num
	int32 data
	---
	bool command_result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Power_sonar_start_Command(const std::shared_ptr<tetra_msgs::srv::PowerSonarCmd::Request> req, 
				std::shared_ptr<tetra_msgs::srv::PowerSonarCmd::Response> res)
{
	bool bResult = false;
	dssp_rs232_power_module_set_Ultrasonic(req->start);
	
  /*
  int32 start
  ---
  bool command_result
  */
	bResult = true;
	res->command_result = bResult;
	return true;
}

int limit_time = 0;
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
	node = rclcpp::Node::make_shared("tetra_interface");

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tetra_battery_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docking_status_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_current_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr conveyor_loadcell_publisher; //conveyor loadcell
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_publisher; //conveyor sensor *2ea
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conveyor_movement_publisher; //conveyor movement
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr power_error_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo_pub;
	std_msgs::msg::Int32 servo;

  node->declare_parameter("time_offset_in_seconds", 0.0);
	time_offset_in_seconds = node->get_parameter("time_offset_in_seconds").as_double();

	rclcpp::Duration duration_time(int32_t(time_offset_in_seconds), uint32_t(time_offset_in_seconds*1000000000));

	//I/O Control Service
  led_service = node->create_service<tetra_msgs::srv::Ledcontrol>("led_cmd", &LEDcontrol_Command);
  ledtoggle_service = node->create_service<tetra_msgs::srv::Ledtogglecontrol>("ledtoggle_cmd", &LEDtoggle_Command);
	turnon_service = node->create_service<tetra_msgs::srv::Toggleon>("turnon_cmd", &TurnOn_Command);
	// Charging Port Control Services
  chargeport_service_on  = node->create_service<std_srvs::srv::Empty>("charging_port_on", &ChargingPortOn);
  chargeport_service_off = node->create_service<std_srvs::srv::Empty>("charging_port_off", &ChargingPortOff);
	//GPIO_Output service
	power_outport_service = node->create_service<tetra_msgs::srv::PowerSetOutport>("Power_outport_cmd", &OutportOnOff);
	power_single_outport_service = node->create_service<tetra_msgs::srv::PowerSetSingleOutport>("Power_single_outport_cmd", &SingleOutportOnOff);
	//GPIO Check service
	power_get_io_service = node->create_service<tetra_msgs::srv::PowerGetIoStatus>("Power_io_status_cmd", &Power_get_IO_Status);

	//Loadcell service
	loadcell_callibration_service = node->create_service<tetra_msgs::srv::LoadcellCallibration>("CAL_cmd", &Loadcell_Callibration_Command);
	//Conveyor Auto Movement Service
	conveyor_auto_movement_service = node->create_service<tetra_msgs::srv::ConveyorAutoMovement>("Auto_Move_cmd", &Conveyor_Auto_Move_Command);
	//Conveyor Manual Movement Service
	conveyor_manual_movement_service = node->create_service<tetra_msgs::srv::ConveyorManualMovement>("Manual_Move_cmd", &Conveyor_Manual_Move_Command);

	//Integral Log Service
	log_service = node->create_service<tetra_msgs::srv::Integrallog>("log_cmd", &Log_Command);

	//Ultrasonic//
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic1_pub = node->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_D_L", 10);
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic2_pub = node->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_R_L", 10);
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic3_pub = node->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_R_R", 10);
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr Ultrasonic4_pub = node->create_publisher<sensor_msgs::msg::Range>("Ultrasonic_D_R", 10);
	
	// PowerSensor parameter read Services
  power_parameter_read_service  = node->create_service<tetra_msgs::srv::PowerParameterRead>("Power_parameter_read_cmd", &Power_parameter_Read_Command);
	conveyor_parameter_read_service  = node->create_service<tetra_msgs::srv::ConveyorParameterRead>("Conveyor_parameter_read_cmd", &Conveyor_parameter_Read_Command);
	power_parameter_write_service  = node->create_service<tetra_msgs::srv::PowerParameterWrite>("Power_parameter_write_cmd", &Power_parameter_Write_Command);
	conveyor_parameter_write_service  = node->create_service<tetra_msgs::srv::ConveyorParameterWrite>("Conveyor_parameter_write_cmd", &Conveyor_parameter_Write_Command);
	//system data read
  conveyor_data_read_service  = node->create_service<tetra_msgs::srv::ConveyorDataRead>("Conveyor_data_read_cmd", &Conveyor_data_Read_Command);
	power_data_read_service  = node->create_service<tetra_msgs::srv::PowerDataRead>("Power_data_read_cmd", &Power_data_Read_Command);
	power_adc_read_service  = node->create_service<tetra_msgs::srv::PowerAdcRead>("Power_adc_read_cmd", &Power_adc_Read_Command);
	power_version_read_service  = node->create_service<tetra_msgs::srv::PowerVersionRead>("Power_version_read_cmd", &Power_version_Read_Command);
	power_sonar_read_service  = node->create_service<tetra_msgs::srv::PowerSonarRead>("Power_sonar_read_cmd", &Power_sonar_Read_Command);

	//battery & status
	tetra_battery_publisher = node->create_publisher<std_msgs::msg::Int32>("tetra_battery", 1);
	std_msgs::msg::Int32 battery_level;
	docking_status_publisher = node->create_publisher<std_msgs::msg::Int32>("docking_status", 1);
	std_msgs::msg::Int32 docking_status;

	//PowerBoard Check
	power_error_publisher = node->create_publisher<std_msgs::msg::Int32>("power_status", 1);
	std_msgs::msg::Int32 power_status;

	//battery Voltage & Current
	battery_voltage_publisher = node->create_publisher<std_msgs::msg::Float64>("battery_voltage", 1);
	std_msgs::msg::Float64 battery_voltage;
	battery_current_publisher = node->create_publisher<std_msgs::msg::Float64>("battery_current", 1);
	std_msgs::msg::Float64 battery_current;
	//GPIO status
	GPIO_pub = node->create_publisher<tetra_msgs::msg::Gpio>("gpio_status", 10);

	//Conveyor Status//
	conveyor_loadcell_publisher = node->create_publisher<std_msgs::msg::Float64>("conveyor_loadcell", 1);
	std_msgs::msg::Float64 conveyor_loadcell;
	conveyor_sensor_publisher = node->create_publisher<std_msgs::msg::Int32>("conveyor_sensor", 1);
	std_msgs::msg::Int32 conveyor_sensor;
	conveyor_movement_publisher = node->create_publisher<std_msgs::msg::Int32>("conveyor_movement", 1);
	std_msgs::msg::Int32 conveyor_movement;

	//Servo On/Off publish
  servo_pub = node->create_publisher<std_msgs::msg::Int32>("Servo_ON",10);

	//POWER Enable service
	power_enable_service = node->create_service<tetra_msgs::srv::PowerSetEnable>("Power_enable_cmd", &PowerEnableOnOff);
	power_single_enable_service = node->create_service<tetra_msgs::srv::PowerSetSingleEnable>("Power_single_enable_cmd", &PowerSingleEnableOnOff);
	power_wheel_enable_service = node->create_service<tetra_msgs::srv::PowerWheelEnable>("Power_wheel_enable_cmd", &PowerWheelEnableOnOff);
	power_sonar_cmd_service = node->create_service<tetra_msgs::srv::PowerSonarCmd>("Power_sonar_start_cmd", &Power_sonar_start_Command);

	//Read Conveyor Option Param Read//
  node->declare_parameter("conveyor_option", true);
	m_bConveyor_option = node->get_parameter("conveyor_option").as_bool();
	printf("##conveyor_option: %d \n", m_bConveyor_option);

	//Ultrasonic Paramter Setting//////////////////////////////////
	char frameid1[] = "/Ultrasonic_Down_Left";
	range_msg1.header.frame_id = frameid1;
	range_msg1.radiation_type = 0; //Ultrasonic
	range_msg1.field_of_view = (60.0/180.0) * M_PI; //
	range_msg1.min_range = Ultrasonic_MIN_range; 
	range_msg1.max_range = Ultrasonic_MAX_range; 

	char frameid2[] = "/Ultrasonic_Rear_Left";
	range_msg2.header.frame_id = frameid2;
	range_msg2.radiation_type = 0; //Ultrasonic
	range_msg2.field_of_view = (60.0/180.0) * M_PI; //
	range_msg2.min_range = Ultrasonic_MIN_range;
	range_msg2.max_range = Ultrasonic_MAX_range;

	char frameid3[] = "/Ultrasonic_Rear_Right";
	range_msg3.header.frame_id = frameid3;
	range_msg3.radiation_type = 0; //Ultrasonic
	range_msg3.field_of_view = (60.0/180.0) * M_PI; //
	range_msg3.min_range = Ultrasonic_MIN_range;
	range_msg3.max_range = Ultrasonic_MAX_range;

	char frameid4[] = "/Ultrasonic_Down_Right";
	range_msg4.header.frame_id = frameid4;
	range_msg4.radiation_type = 0; //Ultrasonic
	range_msg4.field_of_view = (60.0/180.0) * M_PI; //
	range_msg4.min_range = Ultrasonic_MIN_range;
	range_msg4.max_range = Ultrasonic_MAX_range;

	////////////////////////////////////////////////////////////////////////////////////////////
	////sonar range to pointcloud//
	points_1 = node->create_publisher<sensor_msgs::msg::PointCloud2>("range_points_DL", 10);
	points_2 = node->create_publisher<sensor_msgs::msg::PointCloud2>("range_points_RL", 10);
	points_3 = node->create_publisher<sensor_msgs::msg::PointCloud2>("range_points_RR", 10);
	points_4 = node->create_publisher<sensor_msgs::msg::PointCloud2>("range_points_DR", 10);

	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar0_sub = node->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_D_L",10,&RangeToCloud_D_L);
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar1_sub = node->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_R_L",10,&RangeToCloud_R_L);
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar2_sub = node->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_R_R",10,&RangeToCloud_R_R);
	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar3_sub = node->create_subscription<sensor_msgs::msg::Range>("Ultrasonic_D_R",10,&RangeToCloud_D_R);

  rclcpp::Rate loop_rate(30.0); //30Hz Loop
	sprintf(port, "/dev/ttyS0");
	
	//RS232 Connect
	if(dssp_rs232_power_module_create(port, 200) == 0)
	{
		printf("TETRA_Power_rs232 Port Open Success\n");
	}
	else
	{
		printf("TETRA_Power_rs232 Port Open Error!\n");
		return -1;
	}

	//Charge port Enable//
	dssp_rs232_power_module_set_charging_ready(1);
	//Ultrasonic On//
	// dssp_rs232_power_module_set_Ultrasonic(1);

  while(rclcpp::ok())
	{
    rclcpp::spin_some(node);
		// calculate measurement time
		rclcpp::Time measurement_time = rclcpp::Time(0) + duration_time;
	  // m_iPowerCheck = dssp_rs232_power_module_read_battery(&m_dbattery, &m_dVoltage, &m_dCurrent, &m_imode_status, m_iInput, m_iOutput);
		m_iPowerCheck = dssp_rs232_power_module_read_tetra(&m_dbattery, &m_dVoltage, &m_dCurrent, &m_imode_status, m_iInput, m_iOutput, m_dUltrasonic);

		if(m_iPowerCheck < 0)
		{
			printf("!!!! Power Board data read Error(m_iPowerCheck: %d) !!!! \n", m_iPowerCheck);
			if(m_iFlag_PowerCheck_cnt > 10)
			{
				//Error 
				printf("[Error]: Power Board Disconnect !!! \n");
				Error_Log_write("Power Board Disconnect !!!");
				//servo Off
				servo.data = 2;
				servo_pub->publish(servo);
				
			}
			else
			{
				m_iFlag_PowerCheck_cnt++;
			}
			loop_rate.sleep();
			continue;
		}
		else
		{
			m_iFlag_PowerCheck_cnt = 0;
		}
		//add...Power Board Check
		power_status.data = m_iPowerCheck;
		power_error_publisher->publish(power_status);
		m_ibattery_Level = m_dbattery;
		battery_level.data = m_ibattery_Level;
		tetra_battery_publisher->publish(battery_level);
		//battery Voltage
		battery_voltage.data = m_dVoltage;
		battery_voltage_publisher->publish(battery_voltage);
		//battery Current
		battery_current.data = m_dCurrent;
		battery_current_publisher->publish(battery_current);
		//docking_status_publisher
		docking_status.data = m_imode_status;
		docking_status_publisher->publish(docking_status);
				
		//GPIO_status////////////////////////////////////////////
		//Input data
		gpio_msg.input0 = m_iInput[0];
		gpio_msg.input1 = m_iInput[1];
		gpio_msg.input2 = m_iInput[2];
		gpio_msg.input3 = m_iInput[3];
		gpio_msg.input4 = m_iInput[4];
		gpio_msg.input5 = m_iInput[5];
		gpio_msg.input6 = m_iInput[6];
		gpio_msg.input7 = m_iInput[7];
		//Output data
		gpio_msg.output0 = m_iOutput[0];
		gpio_msg.output1 = m_iOutput[1];
		gpio_msg.output2 = m_iOutput[2];
		gpio_msg.output3 = m_iOutput[3];
		gpio_msg.output4 = m_iOutput[4];
		gpio_msg.output5 = m_iOutput[5];
		gpio_msg.output6 = m_iOutput[6];
		gpio_msg.output7 = m_iOutput[7];
		GPIO_pub->publish(gpio_msg);

		//m_dUltrasonic * 4ea
	  //	dssp_rs232_power_module_read_Ultrasonic(m_dUltrasonic);

		if(m_dUltrasonic[0] == 0.0)
			range_msg1.range = Ultrasonic_MAX_range;
		else
			range_msg1.range = m_dUltrasonic[0];

		range_msg1.header.stamp = measurement_time;
		if(m_dUltrasonic[1] == 0.0)
			range_msg2.range = Ultrasonic_MAX_range;
		else
			range_msg2.range = m_dUltrasonic[1];

		range_msg2.header.stamp = measurement_time;
		if(m_dUltrasonic[2] == 0.0)
			range_msg3.range = Ultrasonic_MAX_range;
		else
			range_msg3.range = m_dUltrasonic[2];

		range_msg3.header.stamp = measurement_time;
		if(m_dUltrasonic[3] == 0.0)
			range_msg4.range = Ultrasonic_MAX_range;
		else
			range_msg4.range = m_dUltrasonic[3];
			
		range_msg4.header.stamp = measurement_time;
		//Ultrasonic Publish
		Ultrasonic1_pub->publish(range_msg1);
		Ultrasonic2_pub->publish(range_msg2);
		Ultrasonic3_pub->publish(range_msg3);
		Ultrasonic4_pub->publish(range_msg4);

		if(m_bConveyor_option)
		{
			//Conveyor Loadcell weight
			dssp_rs232_power_module_read_loadcell(&m_dLoadcell_weight);

			conveyor_loadcell.data = m_dLoadcell_weight;
			conveyor_loadcell_publisher->publish(conveyor_loadcell);

			//Conveyor Sensor status
			dssp_rs232_power_module_read_conveyor_sensor(&m_dConveyor_sensor);

			conveyor_sensor.data = m_dConveyor_sensor;
			conveyor_sensor_publisher->publish(conveyor_sensor);

			//Conveyor Movement status
			dssp_rs232_power_module_read_conveyor_movement(&m_iConveyor_movement);

			conveyor_movement.data = m_iConveyor_movement;
			conveyor_movement_publisher->publish(conveyor_movement);
		}

		loop_rate.sleep();
  }

	//Ultrasonic Off//
	dssp_rs232_power_module_set_Ultrasonic(0);
	//RS232 Disconnect
	dssp_rs232_power_module_destroy();

	rclcpp::shutdown();

  return 0;
}
