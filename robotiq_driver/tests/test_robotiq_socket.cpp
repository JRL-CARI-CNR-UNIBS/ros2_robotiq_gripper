#include "robotiq_driver/robotiq_socket.hpp"

int main(int argc, char ** argv)
{

  const std::string address = "192.168.10.2";
  bool auto_calibrate = true;

  robotiq_driver::RobotiqSocket gripper = robotiq_driver::RobotiqSocket(address);

  gripper.connect();
  gripper.activate(auto_calibrate);

  while (true) {
    gripper.move_and_wait_for_pos(255, 10, 10);
    gripper.move_and_wait_for_pos(0, 10, 10);
  }

  return 0;
}
