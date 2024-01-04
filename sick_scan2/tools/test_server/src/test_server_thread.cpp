/*
 * @brief sick_scan2 test_server_thread implements a simple tcp server thread,
 * simulating a lidar device for unittests. It runs a thread to listen
 * and accept tcp connections from clients and generates telegrams to test
 * the sick_scan2 ros driver.
 *
 * Note: sick_scan2 test_server does not implement the functions of lidar sensor,
 * it just implements a simple tcp server, accepting tcp connections from clients
 * and generating telegrams to test the sick_scan2 ros drivers.
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#include <chrono>
#include <thread>

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#include "sick_scan/binPrintf.hpp"
#include "sick_scan/test_server/test_server_thread.h"
#include "sick_scan/test_server/test_server_cola_msg.h"
#include "sick_scan/test_server/test_server_ldmrs_msg.h"

/*!
 * Constructor. The server thread does not start automatically, call start() and stop() to start and stop the server.
 * @param[in] nh ros node handle
 * @param[in] scanner_name scanner type, f.e. "sick_ldmrs"
 * @param[in] ip_port ip port for tcp connections, default: 2112
 */
sick_scan2::test::TestServerThread::TestServerThread(rclcpp::Node::SharedPtr nh, const std::string & scanner_name, int ip_port)
: m_nh(nh), m_scanner_name(scanner_name), m_ip_port(ip_port), m_server_thread(0), m_run_server_thread(false)
{
}

/*!
 * Destructor. Stops the server thread and closes all tcp connections.
 */
sick_scan2::test::TestServerThread::~TestServerThread()
{
}

/*!
 * Starts the server thread, starts to listen and accept tcp connections from clients.
 * @return true on success, false on failure.
 */
bool sick_scan2::test::TestServerThread::start(void)
{
  m_run_server_thread = true;
  m_server_thread = new std::thread(&sick_scan2::test::TestServerThread::run, this);
  return true;
}

/*!
 * Stops the server thread and closes all tcp connections.
 * @return true on success, false on failure.
 */
bool sick_scan2::test::TestServerThread::stop(void)
{
  m_run_server_thread = false;
  if(m_server_thread != 0)
  {
    m_server_thread->join();
    delete m_server_thread;
    m_server_thread = 0;
  }
  return true;
}

/*
 * @brief Thread callback, runs the tcp communication with clients
 */
bool sick_scan2::test::TestServerThread::run(void)
{
  assert(m_nh != 0);
  RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(): starting tcp communication thread, simulating scanner type \"" << m_scanner_name << "\"");
  double send_scan_data_rate = 1/20.0; // frequency to generate and send scan data (default: 20 Hz)
  m_nh->get_parameter("send_scan_data_rate", send_scan_data_rate);
  std::vector<long> scan_data_payload_i;
  m_nh->declare_parameter<std::vector<long>>("scan_data_payload", scan_data_payload_i);
  m_nh->get_parameter("scan_data_payload", scan_data_payload_i);
  std::vector<uint8_t> scan_data_payload;
  scan_data_payload.reserve(scan_data_payload_i.size());
  for(int n = 0; n < scan_data_payload_i.size(); n++)
    scan_data_payload.push_back((uint8_t)(scan_data_payload_i[n] & 0xFF));

  // Create listening socket and wait for connection from a client
  boost::system::error_code error_code;
  boost::asio::io_service ioservice;
  boost::asio::ip::tcp::acceptor tcp_acceptor(ioservice, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), m_ip_port));
  tcp_acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  boost::asio::ip::tcp::socket tcp_client_socket(ioservice);
  RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): waiting for tcp connection on port " << m_ip_port);
  tcp_acceptor.listen();
  tcp_acceptor.accept(tcp_client_socket, error_code);
  if(tcp_client_socket.is_open())
    RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): tcp connection established");

  // Create device specific message handler
  sick_scan2::test::TestServerLidarMsg* msg_handler = 0;
  if(m_scanner_name == "sick_ldmrs")
    msg_handler = new sick_scan2::test::TestServerLDMRSMsg(send_scan_data_rate, scan_data_payload);
  else
    msg_handler = new sick_scan2::test::TestServerColaMsg(send_scan_data_rate, scan_data_payload);
  assert(msg_handler);

  // Run event loop, receive messages, create and send responses and scan data
  std::vector<uint8_t> message_received;
  std::vector<uint8_t> message_response;
  std::vector<uint8_t> scandata_message;
  message_received.reserve(64 * 1024);
  message_response.reserve(64 * 1024);
  bool message_is_binary = false;
  while(rclcpp::ok() && m_run_server_thread && tcp_client_socket.is_open())
  {
    // RCLCPP_DEBUG_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): tcp connection established, running event loop...");
    // Receive message
    if(msg_handler->receiveMessage(tcp_client_socket, message_received, message_is_binary))
    {
      RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): received " << message_received.size() 
        << " byte " << (message_is_binary?"binary":"text") << " message \"" << binDumpVecToString(&message_received, !message_is_binary) << "\"");
      // Generate response
      if(msg_handler->createResponse(message_received, message_is_binary, message_response))
      {
        RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): sending " << message_response.size() 
          << " byte response \"" << binDumpVecToString(&message_response, !message_is_binary) << "\"");
        // Send response
        size_t bytes_written = boost::asio::write(tcp_client_socket, boost::asio::buffer(message_response.data(), message_response.size()), boost::asio::transfer_exactly(message_response.size()), error_code);
        if(error_code || bytes_written < message_response.size())
        {
          RCLCPP_ERROR_STREAM(m_nh->get_logger(), "## ERROR sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): failed to send " << message_response.size() 
            << " byte response \"" << binDumpVecToString(&message_received, !message_is_binary) << "\"");
        }
      }
    }
    else
    {
      // Generate scan data message
      if(msg_handler->createScandata(scandata_message))
      {
        // Send scan data
        size_t bytes_written = boost::asio::write(tcp_client_socket, boost::asio::buffer(scandata_message.data(), scandata_message.size()), boost::asio::transfer_exactly(scandata_message.size()), error_code);
        if(error_code || bytes_written < message_response.size())
        {
          RCLCPP_ERROR_STREAM(m_nh->get_logger(), "## ERROR sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): failed to send " << scandata_message.size() 
            << " byte scan data \"" << binDumpVecToString(&message_received) << "\"");
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Close tcp connection
  RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): closing tcp connection");
  ioservice.stop();
  if(tcp_acceptor.is_open())
  {
    tcp_acceptor.close();
  }
  if(tcp_client_socket.is_open())
  {
    tcp_client_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    tcp_client_socket.close();
  }
  RCLCPP_INFO_STREAM(m_nh->get_logger(), "sick_scan2::test::TestServerThread::run(" << m_scanner_name << "): exiting tcp communication thread");
  m_run_server_thread = false;
  return true;
}