#include "robotiq_driver/robotiq_socket.hpp"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>

using boost::asio::ip::tcp;

namespace robotiq_driver
{
RobotiqSocket::RobotiqSocket(const std::string & address, const int port)
: address_(address),
  active_(false),
  port_(port),
  open_position_(MIN_POSITION),
  closed_position_(MAX_POSITION),
  socket_status_(SocketStatus::DISCONNECTED) {}

RobotiqSocket::~RobotiqSocket()
{
  disconnect();
}

RobotiqSocket::Status RobotiqSocket::connect(const double socket_timeout)
{
  std::cout << "Connecting to gripper at " << address_ << ":" << port_ << std::endl;

  boost::asio::io_context io_context;

  tcp::resolver resolver(io_context);
  tcp::resolver::results_type endpoints = resolver.resolve(address_, std::to_string(port_));

  socket_ = std::make_unique<tcp::socket>(io_context);

  boost::asio::steady_timer timer(io_context);
  timer.expires_after(std::chrono::milliseconds(static_cast<int>(socket_timeout * 1000)));

  bool connected = false;
  std::exception_ptr eptr;

  timer.async_wait(
    [&](const boost::system::error_code & ec) {
      if (!ec) {
        socket_->cancel();
      }
    });

  boost::asio::async_connect(
    *socket_, endpoints, [&](const boost::system::error_code & ec, const tcp::endpoint &) {
      if (!ec) {
        connected = true;
        timer.cancel();
      } else {
        eptr = std::make_exception_ptr(std::runtime_error("Connection failed: " + ec.message()));
      }
    });

  io_context.run();

  if (eptr) {
    socket_status_ = SocketStatus::CONNECTION_ERROR;
    return {SocketStatus::CONNECTION_ERROR, GripperStatus::UNKNOWN};     // If the socket connection timed out, the gripper status is unknown
  }

  if (!connected) {
    socket_status_ = SocketStatus::DISCONNECTED;
    return {SocketStatus::DISCONNECTED, GripperStatus::UNKNOWN};     // If the socket is disconnected, the gripper status is unknown
  }

  std::cout << "Connected to gripper." << std::endl;
  socket_status_ = SocketStatus::CONNECTED;
  return {SocketStatus::CONNECTED, GripperStatus::RESET};   // If the socket is connected, the gripper status is reset
}

void RobotiqSocket::disconnect()
{
  std::cout << "Disconnecting from gripper." << std::endl;
  if (socket_ && socket_->is_open()) {
    socket_->close();
    std::cout << "Disconnected from gripper." << std::endl;
    socket_status_ = SocketStatus::DISCONNECTED;
  }
}

bool RobotiqSocket::send_commands_and_ack(const std::map<std::string, int> & var_dict)
{
  // Construct unique command
  // TODO: togli std::stringstream
  std::stringstream cmd;
  cmd << "SET";
  for (const auto & [variable, value] : var_dict) {
    cmd << " " << variable << " " << value;
  }
  cmd << '\n';    // New line is required for the command to finish

  std::string cmd_str = cmd.str();
  std::vector<char> data(1024);

  // Atomic commands send/receive
  std::lock_guard<std::mutex> lock(mtx_);
  boost::asio::write(*socket_, boost::asio::buffer(cmd_str));
  size_t len = socket_->read_some(boost::asio::buffer(data));

  return is_ack(std::string(data.data(), len));
}

bool RobotiqSocket::send_single_command_and_ack(const std::string & variable, int value)
{
  return send_commands_and_ack({{variable, value}});
}

int RobotiqSocket::get_var(const std::string & variable)
{
  std::string cmd = "GET " + variable + "\n";
  std::vector<char> data(1024);

  // Atomic commands send/receive
  std::lock_guard<std::mutex> lock(mtx_);
  boost::asio::write(*socket_, boost::asio::buffer(cmd));
  size_t len = socket_->read_some(boost::asio::buffer(data));

  std::string response(data.data(), len);
  std::istringstream iss(response);
  std::string var_name;
  int value;
  iss >> var_name >> value;

  if (var_name != variable) {
    throw std::runtime_error("Unexpected response: " + response);
  }

  return value;
}

bool RobotiqSocket::is_ack(const std::string & data)
{
  return data == "ack";
}


RobotiqSocket::Status RobotiqSocket::activate(const double socket_timeout)
{
  if (!is_connected()) {
    std::cout << "Gripper is not connected." << std::endl;
    return {SocketStatus::DISCONNECTED, GripperStatus::UNKNOWN};     // If the gripper is not connected, the gripper status is unknown
  }

  // Reset ACT to 0
  send_single_command_and_ack(ReadWriteVariables::ACT, 0);
  // When setting ACT to one, the Gripper will begin movement to complete its auto-calibration feature
  send_single_command_and_ack(ReadWriteVariables::ACT, 1);

  std::cout << "Waiting for activation" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  // Wait for activation to go through
  while (!is_active()) {
    auto time_now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = time_now - start;

    if (elapsed.count() > socket_timeout) {
      std::cout << "Activation timed out." << std::endl;
      return {SocketStatus::CONNECTED, GripperStatus::ACTIVATION_TIMED_OUT};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "Activated." << std::endl;

  // Auto-calibrate position range even if the activation procedure does it already
  // This allows to set the open_position_ and closed_position_ values
  if (!auto_calibration()) {
    return {SocketStatus::CONNECTED, GripperStatus::CALIBRATION_FAILED}
  }

  std::cout << "Gripper activated." << std::endl;

  return {SocketStatus::CONNECTED, GripperStatus::ACTIVE};
}

bool RobotiqSocket::is_connected()
{
  return socket_status_ == SocketStatus::CONNECTED;
}

bool RobotiqSocket::is_active()
{
  int status = get_var(ReadVariables::STA);
  return status == static_cast<int>(GripperStatus::ACTIVE);
}

// int RobotiqSocket::get_min_position() {
//     return 0; // Example value
// }

// int RobotiqSocket::get_max_position() {
//     return 255; // Example value
// }

int RobotiqSocket::get_open_position()      // min position
{
  return open_position_;
}

int RobotiqSocket::get_closed_position()    // max position
{
  return closed_position_;
}

int RobotiqSocket::get_range()              // delta position
{
  return closed_position_ - open_position_;
}

bool RobotiqSocket::is_open()
{
  return get_current_position() == get_open_position();
}

bool RobotiqSocket::is_closed()
{
  return get_current_position() == get_closed_position();
}

bool RobotiqSocket::is_moving()
{
  return get_var(ReadVariables::OBJ) == static_cast<int>(ObjectStatus::MOVING);
}

bool RobotiqSocket::is_moving_received(const MoveResult & res)
{
  return get_var(ReadVariables::PRE) == res.position;
}

int RobotiqSocket::get_current_position()
{
  return get_var(ReadWriteVariables::POS);
}

int RobotiqSocket::get_current_velocity()
{
  return get_var(ReadWriteVariables::SPE);
}

int RobotiqSocket::get_current_effort()
{
  return get_var(ReadWriteVariables::FOR);
}

bool RobotiqSocket::is_stuck()
{
  return get_var(ReadVariables::OBJ) == static_cast<int>(ObjectStatus::STOPPED_INNER_OBJECT) ||
         get_var(ReadVariables::OBJ) == static_cast<int>(ObjectStatus::STOPPED_OUTER_OBJECT);
}

bool RobotiqSocket::stop_movement()
{
  return send_single_command_and_ack(ReadWriteVariables::GTO, 0);
}

bool RobotiqSocket::reset_velocity()
{
  return send_single_command_and_ack(ReadWriteVariables::SPE, 0);
}

bool RobotiqSocket::auto_calibration(bool log)
{
  if (log) {
    std::cout << "Auto-calibrating gripper." << std::endl;
  }

  // First try to open in case we are holding an object
  MoveResult res = move_and_wait_for_pos(get_open_position(), 64, 1);
  if (res.object_status != ObjectStatus::AT_DEST) {
    std::cout << "Calibration failed opening to start: " << static_cast<int>(res.object_status) <<
      std::endl;
    return false;
  }

  // Try to close as far as possible, and record the number
  res = move_and_wait_for_pos(get_closed_position(), 64, 1);
  if (res.object_status != ObjectStatus::AT_DEST) {
    std::cout << "Calibration failed because of an object: " <<
      static_cast<int>(res.object_status) << std::endl;
    return false;
  }
  assert(res.position <= MAX_POSITION);
  closed_position_ = res.position;

  // Try to open as far as possible, and record the number
  res = move_and_wait_for_pos(get_open_position(), 64, 1);
  if (res.object_status != ObjectStatus::AT_DEST) {
    std::cout << "Calibration failed because of an object: " <<
      static_cast<int>(res.object_status) << std::endl;
    return false;
  }
  assert(res.position >= MIN_POSITION);
  open_position_ = res.position;

  if (log) {
    std::cout << "Gripper auto-calibrated to [" << get_open_position() << ", " <<
      get_closed_position() << "]" << std::endl;
  }

  return true;
}


RobotiqSocket::MoveResult RobotiqSocket::move(int position, int speed, int force)
{
  if (!is_connected()) {
    std::cout << "Gripper is not connected." << std::endl;
    return {false, 0, ObjectStatus::UNKNOWN};
  }

  auto clip_val = [](int min_val, int val, int max_val) {
      return std::max(min_val, std::min(val, max_val));
    };

  int clip_pos = clip_val(MIN_POSITION, position, MAX_POSITION);
  int clip_spe = clip_val(MIN_SPEED, speed, MAX_SPEED);
  int clip_for = clip_val(MIN_FORCE, force, MAX_FORCE);

  // Moves to the given position with the given speed and force
  std::map<std::string, int> var_dict = {
    {ReadWriteVariables::POS, clip_pos},
    {ReadWriteVariables::SPE, clip_spe},
    {ReadWriteVariables::FOR, clip_for},
    {ReadWriteVariables::GTO, 1}
  };

  bool set_ok = send_commands_and_ack(var_dict);
  return {set_ok, clip_pos, static_cast<ObjectStatus>(get_var(ReadVariables::OBJ))};
}

RobotiqSocket::MoveResult RobotiqSocket::move_and_wait_for_pos(int position, int speed, int force)
{
  MoveResult res = move(position, speed, force);
  if (!res.ack) {
    return res;
  }

  // Wait until the gripper acknowledges that it will try to go to the requested position
  while (get_var(ReadVariables::PRE) != res.position) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Wait until not moving
  int cur_obj = get_var(ReadVariables::OBJ);
  while (static_cast<ObjectStatus>(cur_obj) == ObjectStatus::MOVING) {
    cur_obj = get_var(ReadVariables::OBJ);
  }

  // If the gripper is stuck, stop the moving action
  if (static_cast<ObjectStatus>(cur_obj) == ObjectStatus::STOPPED_INNER_OBJECT ||
    static_cast<ObjectStatus>(cur_obj) == ObjectStatus::STOPPED_OUTER_OBJECT)
  {
    send_single_command_and_ack(ReadWriteVariables::GTO, 0);
  }

  // Report the actual position and the object status
  int final_pos = get_var(ReadWriteVariables::POS);

  // Set the final velocity to 0
  send_single_command_and_ack(ReadWriteVariables::SPE, 0);

  return {true, final_pos, static_cast<ObjectStatus>(cur_obj)};
}
} // namespace robotiq_driver
