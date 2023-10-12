#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

//Service_include...
#include "iahrs_msgs/srv/all_data_reset.hpp"
#include "iahrs_msgs/srv/euler_angle_init.hpp"
#include "iahrs_msgs/srv/euler_angle_reset.hpp"
#include "iahrs_msgs/srv/pose_velocity_reset.hpp"
#include "iahrs_msgs/srv/reboot_sensor.hpp"

#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>

#define SERIAL_PORT			"/dev/IMU" ///dev/IMU
#define SERIAL_SPEED		B115200

typedef struct IMU_DATA
{
	double dQuaternion_x = 0.0;
	double dQuaternion_y = 0.0;
	double dQuaternion_z = 0.0;
	double dQuaternion_w = 1.0;

	double dAngular_velocity_x = 0.0;
	double dAngular_velocity_y = 0.0;
	double dAngular_velocity_z = 0.0;
	
	double dLinear_acceleration_x = 0.0;
	double dLinear_acceleration_y = 0.0;
	double dLinear_acceleration_z = 0.0;
    
	double dEuler_angle_Roll = 0.0;
	double dEuler_angle_Pitch = 0.0;
	double dEuler_angle_Yaw = 0.0;

}IMU_DATA;
IMU_DATA _pIMU_data;

int serial_fd = -1;
double time_offset_in_seconds;
double m_dRoll, m_dPitch, m_dYaw;
sensor_msgs::msg::Imu imu_data_msg;
//single_used TF
bool m_bSingle_TF_option = false;

int serial_open()
{
	printf("Try to open serial: %s\n", SERIAL_PORT); 

	serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY);
	if(serial_fd < 0){
		printf("Error unable to open %s\n", SERIAL_PORT);
		return -1;
	}
	printf("%s open success\n", SERIAL_PORT);

	struct termios tio;
	tcgetattr(serial_fd, &tio);
	cfmakeraw(&tio);
	tio.c_cflag = CS8|CLOCAL|CREAD;
	tio.c_iflag &= ~(IXON | IXOFF);
	cfsetspeed(&tio, SERIAL_SPEED);
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 0;

	int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
	if(err != 0){
		printf("Error tcsetattr() function return error\n");
		close(serial_fd);
		serial_fd = -1;
		return -1;
	}
	return 0;
}

static unsigned long GetTickCount() 
{
	struct timespec ts;
	
	clock_gettime(CLOCK_MONOTONIC, &ts);

	return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

int SendRecv(const char* command, double* returned_data, int data_length)
{
	#define COMM_RECV_TIMEOUT	30	

	char temp_buff[256];
	read(serial_fd, temp_buff, 256);

	int command_len = strlen(command);
	int n = write(serial_fd, command, command_len);

	if(n < 0) return -1;

	const int buff_size = 1024;
	int  recv_len = 0;
	char recv_buff[buff_size + 1];

	unsigned long time_start = GetTickCount();

	while(recv_len < buff_size){
		int n = read(serial_fd, recv_buff + recv_len, buff_size - recv_len);
		if(n < 0) 
		{
			return -1;
		}
		else if(n == 0) 
		{
			usleep(1000);
		}
		else if(n > 0) 
		{
			recv_len += n;

			if(recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') 
			{
				break;
			}
		}

		unsigned long time_current = GetTickCount();
		unsigned long time_delta = time_current - time_start;

		if(time_delta >= COMM_RECV_TIMEOUT) break;
	}
	recv_buff[recv_len] = '\0';

	if(recv_len > 0) 
	{
		if(recv_buff[0] == '!') 
		{
			return -1;
		}
	}

	if(strncmp(command, recv_buff, command_len - 1) == 0){
		if(recv_buff[command_len - 1] == '='){
			int data_count = 0;

			char* p = &recv_buff[command_len];
			char* pp = NULL;

			for(int i = 0; i < data_length; i++) 
			{
				if(p[0] == '0' && p[1] == 'x') 
				{
					returned_data[i] = strtol(p+2, &pp, 16);
					data_count++;
				}
				else 
				{
					returned_data[i] = strtod(p, &pp);
					data_count++;
				}

				if(*pp == ',') 
				{
					p = pp + 1;
				}
				else 
				{
					break;
				}
			}
			return data_count;
		}
	}
	return 0;
}

//ROS Service Callback////////////////////////////////////////////////////////////////////////
bool All_Data_Reset_Command(const std::shared_ptr<iahrs_msgs::srv::AllDataReset::Request> req,
					    	std::shared_ptr<iahrs_msgs::srv::AllDataReset::Response> res)
{
	bool bResult = false;

	double dSend_Data[10];
	SendRecv("rc\n", dSend_Data, 10);

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Euler_Angle_Init_Command(const std::shared_ptr<iahrs_msgs::srv::EulerAngleInit::Request> req, 
					    	  std::shared_ptr<iahrs_msgs::srv::EulerAngleInit::Response> res)
{
	bool bResult = false;

	double dSend_Data[10];
	SendRecv("za\n", dSend_Data, 10);

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Euler_Angle_Reset_Command(const std::shared_ptr<iahrs_msgs::srv::EulerAngleReset::Request> req, 
					    	   std::shared_ptr<iahrs_msgs::srv::EulerAngleReset::Response> res)
{
	bool bResult = false;

	double dSend_Data[10];
	SendRecv("ra\n", dSend_Data, 10);

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Pose_Velocity_Reset_Command(const std::shared_ptr<iahrs_msgs::srv::PoseVelocityReset::Request> req, 
					    	     std::shared_ptr<iahrs_msgs::srv::PoseVelocityReset::Response> res)
{
	bool bResult = false;

	double dSend_Data[10];
	SendRecv("rp\n", dSend_Data, 10);

	bResult = true;
	res->command_result = bResult;
	return true;
}

bool Reboot_Sensor_Command(const std::shared_ptr<iahrs_msgs::srv::RebootSensor::Request> req, 
					       std::shared_ptr<iahrs_msgs::srv::RebootSensor::Response> res)
{
	bool bResult = false;

	double dSend_Data[10];
	SendRecv("rd\n", dSend_Data, 10);

	bResult = true;
	res->command_result = bResult;
	return true;
}

/***main loop...*****/
int main(int argc, char** argv)
{
	// signal(SIGINT,my_handler);
  rclcpp::init(argc, argv);
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("iahrs_driver");

  std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  tf2::Transform transform;
	
	// These values do not need to be converted
	imu_data_msg.linear_acceleration_covariance[0] = 0.0064;
	imu_data_msg.linear_acceleration_covariance[4] = 0.0063;
	imu_data_msg.linear_acceleration_covariance[8] = 0.0064;

	imu_data_msg.angular_velocity_covariance[0] = 0.032*(M_PI/180.0);
	imu_data_msg.angular_velocity_covariance[4] = 0.028*(M_PI/180.0);
	imu_data_msg.angular_velocity_covariance[8] = 0.006*(M_PI/180.0);

	imu_data_msg.orientation_covariance[0] = 0.013*(M_PI/180.0);
	imu_data_msg.orientation_covariance[4] = 0.011*(M_PI/180.0);
	imu_data_msg.orientation_covariance[8] = 0.006*(M_PI/180.0);
	
	auto imu_data_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

	//IMU Service///////////////////////////////////////////////////////////////////////////////////////////////
  auto all_data_reset_service = node->create_service<iahrs_msgs::srv::AllDataReset>("all_data_reset_cmd", &All_Data_Reset_Command);
	auto euler_angle_init_service = node->create_service<iahrs_msgs::srv::EulerAngleInit>("euler_angle_init_cmd", &Euler_Angle_Init_Command);
	auto euler_angle_reset_service = node->create_service<iahrs_msgs::srv::EulerAngleReset>("euler_angle_reset_cmd", &Euler_Angle_Reset_Command);
	auto pose_velocity_reset_service = node->create_service<iahrs_msgs::srv::PoseVelocityReset>("pose_velocity_reset_cmd", &Pose_Velocity_Reset_Command);
	auto reboot_sensor_service = node->create_service<iahrs_msgs::srv::RebootSensor>("reboot_sensor_cmd", &Reboot_Sensor_Command);

  node->declare_parameter("m_bSingle_TF_option", true);
  m_bSingle_TF_option = node->get_parameter("m_bSingle_TF_option").as_bool();
	printf("##m_bSingle_TF_option: %d \n", m_bSingle_TF_option);

  rclcpp::WallRate loop_rate(10); //HZ
	serial_open();

	double dSend_Data[10];
	SendRecv("za\n", dSend_Data, 10);	// Euler Angle -> '0.0' Reset
	usleep(10000);

	while(rclcpp::ok())
	{
		rclcpp::spin_some(node);

		if(serial_fd >= 0) 
		{
			const int max_data = 10;
			double data[max_data];
			int no_data = 0;

			no_data = SendRecv("g\n", data, max_data);	// Read angular_velocity _ wx, wy, wz 
			if(no_data >= 3) 
			{
				// angular_velocity
				imu_data_msg.angular_velocity.x = _pIMU_data.dAngular_velocity_x = data[0]*(M_PI/180.0);
				imu_data_msg.angular_velocity.y = _pIMU_data.dAngular_velocity_y = data[1]*(M_PI/180.0);
				imu_data_msg.angular_velocity.z = _pIMU_data.dAngular_velocity_z = data[2]*(M_PI/180.0);
			}

			no_data = SendRecv("a\n", data, max_data);	// Read linear_acceleration 	unit: m/s^2
			if (no_data >= 3) 
			{
				//// linear_acceleration   g to m/s^2
				imu_data_msg.linear_acceleration.x = _pIMU_data.dLinear_acceleration_x = data[0] * 9.80665;
				imu_data_msg.linear_acceleration.y = _pIMU_data.dLinear_acceleration_y = data[1] * 9.80665;
				imu_data_msg.linear_acceleration.z = _pIMU_data.dLinear_acceleration_z = data[2] * 9.80665;
			}
			
			no_data = SendRecv("e\n", data, max_data);	// Read Euler angle
			if (no_data >= 3) 
			{
				// Euler _ rad
				_pIMU_data.dEuler_angle_Roll  = data[0]*(M_PI/180.0);
				_pIMU_data.dEuler_angle_Pitch = data[1]*(M_PI/180.0);
				_pIMU_data.dEuler_angle_Yaw	  = data[2]*(M_PI/180.0);
			}

			tf2::Quaternion orientation;
			orientation.setRPY(_pIMU_data.dEuler_angle_Roll , _pIMU_data.dEuler_angle_Pitch, _pIMU_data.dEuler_angle_Yaw);
			// orientation
			imu_data_msg.orientation.x = orientation.x();
			imu_data_msg.orientation.y = orientation.y();
			imu_data_msg.orientation.z = orientation.z();
			imu_data_msg.orientation.w = orientation.w();
			
			imu_data_msg.header.stamp = node->get_clock()->now();
			imu_data_msg.header.frame_id = "imu_link";  // "imu_link"

			// publish the IMU data
			imu_data_pub->publish(imu_data_msg);

			//Publish tf
			if(m_bSingle_TF_option)
			{
				transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.2));
				tf2::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_link"));
        geometry_msgs::msg::TransformStamped baseToImu;
				baseToImu.header.stamp = node->get_clock()->now();
        baseToImu.header.frame_id = "base_link";
        baseToImu.child_frame_id = "imu_link";
        baseToImu.transform = tf2::toMsg(transform);
				br->sendTransform(baseToImu);
			}
		}
		loop_rate.sleep();
	}
	close (serial_fd);
	rclcpp::shutdown();
	return 0;
}
