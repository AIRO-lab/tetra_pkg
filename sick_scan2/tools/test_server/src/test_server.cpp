/*
 * @brief sick_scan2 test_server implements a simple tcp server,
 * simulating a lidar device for unittests.
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

#include "sick_scan/test_server/test_server_thread.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization, pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("sick_test_server", "", node_options);
  setMainNode(node);
  std::string scanner_name = "undefined";
  int port = 2112;
  double send_scan_data_rate = 1/20.0; // frequency to generate and send scan data (default: 20 Hz)
  node->declare_parameter<std::string>("scanner_name", scanner_name);
  node->declare_parameter<int>("port", port);
  node->declare_parameter<double>("send_scan_data_rate", send_scan_data_rate);
  node->get_parameter("scanner_name", scanner_name);
  node->get_parameter("port", port);
  node->get_parameter("send_scan_data_rate", send_scan_data_rate);
  RCLCPP_INFO(node->get_logger(), "sick_scan2_test_server started, simulating scanner type \"%s\" on tcp port %d", scanner_name.c_str(), port);

  // Start a tcp test server to simulate a lidar device
  sick_scan2::test::TestServerThread test_server_thread(node, scanner_name, port);
  test_server_thread.start();

  // Run ros event loop
  RCLCPP_INFO(node->get_logger(), "sick_scan2_test_server is running event loop");
  rclcpp::spin(node);

  // Cleanup and exit
  RCLCPP_INFO(node->get_logger(), "sick_scan2_test_server finished.");
  test_server_thread.stop();
  RCLCPP_INFO(node->get_logger(), "sick_scan2_test_server exits.");
  return 0;
}
