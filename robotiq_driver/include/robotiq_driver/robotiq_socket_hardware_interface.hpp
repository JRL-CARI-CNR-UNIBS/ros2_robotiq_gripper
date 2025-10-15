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

#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <robotiq_driver/robotiq_socket.hpp>


#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotiq_driver
{
class RobotiqSocketHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotiqSocketHardwareInterface)

  /**
   * Default constructor.
   */
  RobotiqSocketHardwareInterface();

  ~RobotiqSocketHardwareInterface();

  /**
   * Initialization of the hardware interface from data parsed from the
   * robot's URDF.
   * @param hardware_info Structure with data from URDF.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Connect to the hardware.
   * @param previous_state The previous state.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * This method is invoked when the hardware is connected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method is invoked when the hardware is disconnected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Read data from the hardware.
   */
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * Write data to hardware.
   */
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // Constants
  static constexpr double DEFAULT_TIMEOUT = 30.0 * 1000;                           // Connection timeout [ms]
  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();  // Uninitialized command
  static constexpr auto GRIPPER_COMMS_LOOP_PERIOD = std::chrono::milliseconds{100}; // Communication loop period [ms]
  static constexpr int NUM_STATE_INTERFACES = 2;
  static constexpr int NUM_COMMAND_INTERFACES = 3;
  // static constexpr double GRIPPER_MAX_POSITION = 0.085;                            // [m]
  // static constexpr double GRIPPER_MAX_SPEED = 0.150;                               // [m/s]
  static constexpr double GRIPPER_MAX_FORCE = 235;                                 // [N]
  
  // Parameters (read from ros2 control XACRO)
  double gripper_closed_pos_rad_ = 0.0; // unused when using GRIPPER_MAX_POSITION
  double gripper_max_speed_ = 0.150;
  bool activate_gripper_by_default_ = false;

  // Logger
  const rclcpp::Logger LOGGER = rclcpp::get_logger("RobotiqSocketHardwareInterface");

  // Robotiq Socket gripper
  std::unique_ptr<robotiq_driver::RobotiqSocket> gripper_;

  // State interfaces (use radians for compatibility with robot state publisher)
  double gripper_position_ = 0.0;   // [rad]
  double gripper_velocity_ = 0.0;   // [rad/s]
  // double gripper_effort_   = 0.0;   // [N] DOES NOT MAKE SENSE: NO FORCE SENSOR INSTALLED

  // Command interfaces (use meters for convenience when issuing commands)
  double gripper_position_command_ = 0.0; // [m]
  double gripper_velocity_command_ = 0.0; // [m/s]
  double gripper_effort_command_   = 0.0; // [N]

  // Command interfaces to reactivate the gripper
  double reactivate_gripper_command_= 0.0;
  double reactivate_gripper_response_ = 0.0;

  // Atomic variables read/set by the read/write methods and the communication thread
  std::atomic<uint8_t> write_command_;
  std::atomic<uint8_t> write_command_previous_;  
  std::atomic<uint8_t> write_force_;
  std::atomic<uint8_t> write_speed_;
  std::atomic<uint8_t> gripper_current_position_int_;
  std::atomic<uint8_t> gripper_current_velocity_int_;
  // std::atomic<uint8_t> gripper_current_effort_int_; DOES NOT MAKE SENSE: NO FORCE SENSOR INSTALLED

  // Atomic variables read by the read method and read/set by the communication thread
  std::atomic<bool> reactivate_gripper_async_cmd_;
  std::atomic<std::optional<bool>> reactivate_gripper_async_response_;

  // We use a thread to read/write to the driver so that we dont block the hardware_interface read/write.
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_;
  void communication_task();
};

}  // namespace robotiq_driver
