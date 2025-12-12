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
  gripper_max_speed_ = info_.hardware_parameters.count("gripper_max_speed") ?
                          stod(info_.hardware_parameters["gripper_max_speed"]) :
                          0.150;


  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = NO_NEW_CMD_;
  gripper_effort_command_ = NO_NEW_CMD_;

  // set reactivation variables to default values
  reactivate_gripper_command_ = NO_NEW_CMD_;
  reactivate_gripper_async_cmd_.store(false);

  const hardware_interface::ComponentInfo& joint = info_.joints[0];

  // Command interfaces: position, velocity, and effort.
  if (joint.command_interfaces.size() != NUM_COMMAND_INTERFACES)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. %d expected.", joint.name.c_str(),
                 joint.command_interfaces.size(), NUM_COMMAND_INTERFACES);
    return CallbackReturn::ERROR;
  }
  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }
  if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    return CallbackReturn::ERROR;
  }
  if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
    return CallbackReturn::ERROR;
  }

  // There are three state interfaces: position, velocity, and effort.
  if (joint.state_interfaces.size() != NUM_STATE_INTERFACES)
  {
    RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. %d expected.", joint.name.c_str(),
                 joint.state_interfaces.size(), NUM_STATE_INTERFACES);
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < NUM_STATE_INTERFACES; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION
          || joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY
          // || joint.state_interfaces[i].name == hardware_interface::HW_IF_EFFORT // DOES NOT MAKE SENSE: NO FORCE SENSOR INSTALLED
    ))
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces[i].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY
                   //, hardware_interface::HW_IF_EFFORT // DOES NOT MAKE SENSE: NO FORCE SENSOR INSTALLED
      );
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotiqSocketHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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
    RCLCPP_INFO(LOGGER, "activate_gripper_by_default parameter found. Using value: %s", activate_gripper_by_default_ ? "true" : "false");
  }
  else {
    RCLCPP_WARN(LOGGER, "activate_gripper_by_default parameter not found. Using default value.");
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

  // DOES NOT MAKE SENSE: NO FORCE SENSOR INSTALLED
  // state_interfaces.emplace_back(
  //     hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &gripper_effort_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotiqSocketHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(LOGGER, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_command_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &gripper_effort_command_));

  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_velocity", &gripper_velocity_command_));
  gripper_velocity_command_ = info_.hardware_parameters.count("gripper_speed_multiplier") ?
                              info_.hardware_parameters.count("gripper_speed_multiplier") :
                              1.0;

  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_effort", &gripper_effort_command_));
  gripper_effort_command_ = info_.hardware_parameters.count("gripper_force_multiplier") ?
                            info_.hardware_parameters.count("gripper_force_multiplier") :
                            1.0;

  // Command interface for reactivating the gripper
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

  RCLCPP_INFO(LOGGER, "Activating Robotiq Gripper...");
  if (activate_gripper_by_default_){
    RobotiqSocket::Status status = gripper_->activate();
    if(status.socket_status != RobotiqSocket::SocketStatus::CONNECTED || status.gripper_status != RobotiqSocket::GripperStatus::ACTIVE){
      RCLCPP_ERROR(LOGGER, "Failed to activate the Robotiq gripper: %d", static_cast<int>(status.gripper_status));
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(LOGGER, "Activation completed!");

  write_command_.store(uint8_t(gripper_->get_current_position()));
  write_command_previous_.store(write_command_.load());
  write_speed_.store(uint8_t(0));
  write_force_.store(uint8_t(0));

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
  // Gripper position in rad (small-angle approximation)
  gripper_position_ = gripper_closed_pos_rad_ * 
    (gripper_current_position_int_.load() - gripper_->get_open_position()) / gripper_->get_range();

  // Gripper velocity in rad/s (small-angle approximation) 
  gripper_velocity_ = gripper_current_velocity_int_.load() * gripper_max_speed_ / RobotiqSocket::MAX_SPEED *
    gripper_closed_pos_rad_ / gripper_closed_pos_rad_;

  // Gripper effort in N -> DOES NOT MAKE SENSE: NO FORCE SENSOR INSTALLED
  // gripper_effort_ = gripper_current_effort_int_.load() * GRIPPER_MAX_FORCE / RobotiqSocket::MAX_FORCE;

  // Gripper activation async
  if (!std::isnan(reactivate_gripper_command_))
  {
    RCLCPP_INFO(LOGGER, "Sending gripper reactivation request.");
    reactivate_gripper_async_cmd_.store(true);
    reactivate_gripper_command_ = NO_NEW_CMD_;
  }

  if (reactivate_gripper_async_response_.load().has_value())
  {
    reactivate_gripper_response_ = reactivate_gripper_async_response_.load().value();
    reactivate_gripper_async_response_.store(std::nullopt);
  }

  RCLCPP_DEBUG(LOGGER, "Gripper position: %.3f m, velocity: %.3f m/s", gripper_position_, gripper_velocity_);


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqSocketHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  // Gripper position command in ticks [0-255] from gripper_position_command_ in meters [0-0.085]
  
  double gripper_position_cmd = (gripper_position_command_ / gripper_closed_pos_rad_) * gripper_->get_range() + gripper_->get_open_position();
  gripper_position_cmd = std::max(std::min(gripper_position_cmd, 
                                           static_cast<double>(RobotiqSocket::MAX_POSITION)), 
                                  static_cast<double>(RobotiqSocket::MIN_POSITION));

  // Gripper velocity command in ticks/s [0-255] from gripper_velocity_command_ in m/s [0-0.150]
  double gripper_velocity_cmd = RobotiqSocket::MAX_SPEED * std::clamp(fabs(gripper_velocity_command_) / gripper_max_speed_, 0.0, 1.0);
  
  // Gripper effort command in ticks [0-255] from gripper_effort_command_ in N [0-235]
  double gripper_effort_cmd = RobotiqSocket::MAX_FORCE * std::clamp(fabs(gripper_effort_command_) / GRIPPER_MAX_FORCE, 0.0, 1.0);
  
  write_command_.store(uint8_t(gripper_position_cmd));
  write_speed_.store(uint8_t(gripper_velocity_cmd));
  write_force_.store(uint8_t(gripper_effort_cmd));

  return hardware_interface::return_type::OK;
}

void RobotiqSocketHardwareInterface::communication_task()
{
  while (communication_thread_is_running_.load())
  {
    // Write command to gripper
    if(!gripper_->is_moving())
    {
      // Reset the gripper velocity
      gripper_->reset_velocity();

      // Re-activate the gripper if needed
      if (reactivate_gripper_async_cmd_.load()){
        gripper_->activate();
        reactivate_gripper_async_cmd_.store(false);
        reactivate_gripper_async_response_.store(true);
      }

      // Move the gripper
      if(write_command_.load() != write_command_previous_.load()){
        auto res = gripper_->move(write_command_.load(), write_speed_.load(), write_force_.load());
        write_command_previous_.store(write_command_.load());
        double write_command_meters = gripper_closed_pos_rad_ * 
          (write_command_.load() - gripper_->get_open_position()) / gripper_->get_range();
        RCLCPP_INFO(LOGGER, "Gripper moving to position: %.3f m", write_command_meters);
        while (!gripper_->is_moving_received(res)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    }
    else{
      if(gripper_->is_stuck()){
        RCLCPP_WARN(LOGGER, "Gripper is stuck. Stopping the movement.");
        gripper_->stop_movement();
        RCLCPP_WARN(LOGGER, "Gripper stopped.");
      }
    }
    gripper_current_position_int_.store(gripper_->get_current_position());
    gripper_current_velocity_int_.store(gripper_->get_current_velocity());

    std::this_thread::sleep_for(GRIPPER_COMMS_LOOP_PERIOD);
  }
}

}  // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqSocketHardwareInterface, hardware_interface::SystemInterface)
