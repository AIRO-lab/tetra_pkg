/*
 * Copyright (C) 2015, DFKI GmbH
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
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *  Created on: 20.11.2015
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *
 *  Modified and ported to ROS2: 02.10.2020 by Ing.-Buero Dr. Michael Lehning, Hildesheim
 *  Integrated into sick_scan2:  13.10.2020 by Ing.-Buero Dr. Michael Lehning, Hildesheim
 */
#if defined LDMRS_SUPPORT && LDMRS_SUPPORT > 0

#include <sick_scan/ldmrs/sick_ldmrs_node.h>

sick_scan::SickLdmrsNode::SickLdmrsNode()
: m_nh(0), m_diagnostics(0), m_manager(0), m_app(0), m_ldmrs(0)
{
}

sick_scan::SickLdmrsNode::~SickLdmrsNode()
{
  delete(m_manager);
  delete(m_app);
  delete(m_ldmrs);
}

int sick_scan::SickLdmrsNode::init(rclcpp::Node::SharedPtr nh, const std::string & hostName, const std::string & frameId)
{
  m_nh = nh;
  m_diagnostics = boost::make_shared<diagnostic_updater::Updater>(m_nh);

  // The MRS-App connects to an MRS, reads its configuration and receives all incoming data.
  // First, create the manager object. The manager handles devices, collects
  // device data and forwards it to the application(s).
  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Creating the manager.");
  m_manager = new Manager();

  // Add the application. As the devices may send configuration data only once
  // at startup, the applications must be present before the devices are
  // started.
  // Sourcetype type;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Adding the application SickLDMRS.");
  std::string name = "Sick LDMRS ROS Driver App";
  UINT16 id = 1356;

  m_app = new sick_ldmrs_driver::SickLDMRS(m_nh, m_manager, m_diagnostics);
  m_app->setApplicationName(name);

  bool result = m_manager->addApplication(m_app, id);
  if (result == false)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Failed to add application " << name << ", aborting!");
    return sick_scan::ExitError;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Application is running.");

  //
  // Add and run the sensor
  //
  // The MRS device could be configured like this:
  // m_weWantScanData:          true
  // m_weWantObjectData:        true
  // m_weWantFieldData:         false
  // m_weWantScanDataFromSopas: false
  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Adding the LDMRS device.");
  m_ldmrs = new devices::LDMRS(m_manager);
  m_ldmrs->setWeWantObjectData(true);
  std::string hostname;
  sick_ldmrs_driver::param<std::string>(m_nh, "hostname", hostname, "192.168.0.1");
  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Set IP address to " << hostname);
  m_ldmrs->setIpAddress(hostname);
  name = "LDMRS-1";
  id = 1;
  result = m_manager->addAndRunDevice(m_ldmrs, name, id);
  if (result == false)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "Failed to add device " << name << ", aborting!");
    return sick_scan::ExitError;
  }

  std::string serial_number = m_ldmrs->getSerialNumber();
  m_diagnostics->setHardwareID(serial_number);
  m_diagnostics->setPeriod(0.1); // publish diagnostic messages with 10 Hz

  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "LD-MRS Firmware version is " << m_ldmrs->getFirmwareVersion());

  // we need to initialize the app after setting up the ldmrs device
  m_app->init();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_ldmrs_driver"), "sick_ldmrs_driver is initialized.");
  return sick_scan::ExitSuccess;
}

int sick_scan::SickLdmrsNode::run(void)
{
  // rclcpp::Rate r(std::chrono::nanoseconds((int64_t)(0.1 * 1.0e9))); // ros::Rate r(10.0);
  // while (rclcpp::ok()) // while (ros::ok())
  // {
  //   rclcpp::spin_some(m_nh); // ros::spinOnce();
  //   m_diagnostics->force_update(); // m_diagnostics->update();
  //   r.sleep();
  // }
  rclcpp::spin(m_nh);
  return sick_scan::ExitSuccess;
}

#endif // LDMRS_SUPPORT && LDMRS_SUPPORT > 0
