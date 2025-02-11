// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <robotiq_driver/robotiq_socket_hardware_interface.hpp>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

namespace robotiq_driver
{
RobotiqSocketHardwareInterface::RobotiqSocketHardwareInterface() :
  LOGGER(rclcpp::get_logger("RobotiqSocketHardwareInterface"))
{
}

RobotiqSocketHardwareInterface::~RobotiqSocketHardwareInterface()
{
  gripper_ = nullptr;
}

hardware_interface::CallbackReturn RobotiqSocketHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(LOGGER, "on_init");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read parameters.
  gripper_closed_pos_rad_ = stod(info_.hardware_parameters["gripper_closed_position"]);

  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = NO_NEW_CMD_;
  gripper_force_command_ = NO_NEW_CMD_;

  // reactivate_gripper_async_cmd_.store(false);

  const hardware_interface::ComponentInfo& joint = info_.joints[0];

  // There is one command interface: position.
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                 joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }

  // There are two state interfaces: position and velocity.
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                 joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces[i].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotiqSocketHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(LOGGER, "on_configure");
  
  double connection_timeout = DEFAULT_TIMEOUT;
  if(info_.hardware_parameters.find("connection_timeout") != info_.hardware_parameters.end()){
    connection_timeout = std::stod(info_.hardware_parameters["connection_timeout"]);
    if(connection_timeout <= 0){
      RCLCPP_WARN(LOGGER, "Invalid connection_timeout value. Using default value.");
      connection_timeout = DEFAULT_TIMEOUT;
    }
  }

  if(info_.hardware_parameters.find("activate_gripper_by_default") != info_.hardware_parameters.end()){
    activate_gripper_by_default_ = std::stoi(info_.hardware_parameters["activate_gripper_by_default"]);
  }
  if(info_.hardware_parameters.find("auto_calibrate") != info_.hardware_parameters.end()){
    auto_calibrate_ = std::stoi(info_.hardware_parameters["auto_calibrate"]);
  }

  gripper_ = std::make_unique<RobotiqSocket>(info_.hardware_parameters["address"], 
                                            std::stoi(info_.hardware_parameters["port"]));
  
  RobotiqSocket::Status status = gripper_->connect(connection_timeout);

  if (status.socket_status != RobotiqSocket::SocketStatus::CONNECTED)
  {
    RCLCPP_ERROR(LOGGER, "Cannot configure the Robotiq Socket gripper: %d", static_cast<int>(status.socket_status));
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotiqSocketHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(LOGGER, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotiqSocketHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(LOGGER, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_velocity", &gripper_velocity_command_));
  gripper_velocity_command_ = info_.hardware_parameters.count("gripper_speed_multiplier") ?
                           info_.hardware_parameters.count("gripper_speed_multiplier") :
                           1.0;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_effort", &gripper_force_command_));
  gripper_force_command_ = info_.hardware_parameters.count("gripper_force_multiplier") ?
                       info_.hardware_parameters.count("gripper_force_multiplier") :
                       1.0;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("reactivate_gripper", "reactivate_gripper_cmd", &reactivate_gripper_command_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "reactivate_gripper", "reactivate_gripper_response", &reactivate_gripper_response_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
RobotiqSocketHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(LOGGER, "on_activate");

  // set some default values for joints
  if (std::isnan(gripper_position_)){
    gripper_position_ = 0;
    gripper_velocity_ = 0;
    gripper_position_command_ = 0;
  }

  if (activate_gripper_by_default_){
    RobotiqSocket::Status status = gripper_->activate(auto_calibrate_);
    if(status.socket_status != RobotiqSocket::SocketStatus::CONNECTED || status.gripper_status != RobotiqSocket::GripperStatus::ACTIVE){
      RCLCPP_ERROR(LOGGER, "Failed to activate the Robotiq gripper: %d", static_cast<int>(status.gripper_status));
      return CallbackReturn::ERROR;
    }
  }

  // Start communication thread
  communication_thread_is_running_.store(true);
  communication_thread_ = std::thread([this] { this->communication_task(); });

  RCLCPP_INFO(LOGGER, "Robotiq Gripper successfully activated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RobotiqSocketHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(LOGGER, "on_deactivate");

  // Stop communication thread
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
    communication_thread_.join();

  // Disconnect from the gripper
  gripper_->disconnect();
  
  RCLCPP_INFO(LOGGER, "Robotiq Gripper successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqSocketHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/)
{
  gripper_position_ = gripper_closed_pos_rad_ *
    (gripper_current_state_.load() - gripper_->get_open_position()) / gripper_->get_range();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqSocketHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  double gripper_pos = (gripper_position_command_ / gripper_closed_pos_rad_) * gripper_->get_range() + gripper_->get_open_position();
  gripper_pos = std::max(std::min(gripper_pos, 255.0), 0.0);
  gripper_velocity_command_ = GRIPPER_MAX_SPEED * std::clamp(fabs(gripper_velocity_command_) / GRIPPER_MAX_SPEED, 0.0, 1.0);
  gripper_force_command_ = GRIPPER_MAX_FORCE * std::clamp(fabs(gripper_force_command_) / GRIPPER_MAX_FORCE, 0.0, 1.0);
  
  write_command_.store(uint8_t(gripper_pos));
  write_speed_.store(uint8_t(gripper_velocity_command_));
  write_force_.store(uint8_t(gripper_force_command_));

  return hardware_interface::return_type::OK;
}

void RobotiqSocketHardwareInterface::communication_task()
{
  while (communication_thread_is_running_.load())
  {
    // Write command to gripper
    if(!gripper_->is_moving())
    {
      auto res = gripper_->move(write_command_.load(), write_speed_.load(), write_force_.load());
      while (gripper_->is_moving_received(res)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    // Read state from gripper
    gripper_current_state_ = gripper_->get_current_position();

    std::this_thread::sleep_for(GRIPPER_COMMS_LOOP_PERIOD);
  }
}

}  // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqSocketHardwareInterface, hardware_interface::SystemInterface)
