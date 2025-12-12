#pragma once

#include <string>
#include <utility>
#include <memory>
#include <boost/asio.hpp>
#include <map>

using boost::asio::ip::tcp;

namespace robotiq_driver
{
class RobotiqSocket {

public:
    enum class SocketStatus {
        DISCONNECTED = 0,
        CONNECTED = 1,
        CONNECTION_ERROR = 2,
    };

    enum class GripperStatus {       // Status values from 0 to 3 are mapped to the gripper firmware
        RESET = 0,                   // This value is mapped to the gripper firmware, but not used
        ACTIVE = 3,                  // This value is mapped to the gripper firmware that returns this value when the activation is successful
        UNKNOWN = 4,                 // This value is NOT MAPPED TO the gripper firmware
        ACTIVATION_TIMED_OUT = 5,    // This value is NOT MAPPED TO the gripper firmware
        CALIBRATION_FAILED = 6,      // This value is NOT MAPPED TO the gripper firmware
    };

    enum class ObjectStatus {
        MOVING = 0, // The firmware will return this value when the gripper is moving
        STOPPED_OUTER_OBJECT = 1,
        STOPPED_INNER_OBJECT = 2,
        AT_DEST = 3, // The firmware will return this value when the gripper is at the requested position
        UNKNOWN = 4, // This value is NOT MAPPED TO the gripper firmware
    };

    struct Status {
        SocketStatus socket_status;
        GripperStatus gripper_status;
    };

    struct MoveResult {
        bool ack;
        int position;
        ObjectStatus object_status;
    };

private:
    std::mutex mtx_;

    bool is_ack(const std::string& data);
    bool send_commands_and_ack(const std::map<std::string, int>& var_dict);
    bool send_single_command_and_ack(const std::string& variable, int value);
    int get_var(const std::string& variable);
    bool auto_calibration(bool log = true);

protected:
    std::string address_;
    bool active_;
    int port_;
    int open_position_;
    int closed_position_;
    std::unique_ptr<tcp::socket> socket_;
    SocketStatus socket_status_;

    struct ReadWriteVariables {
        static constexpr const char* ACT = "ACT";  // act : activate (1 while activated, can be reset to clear fault status)
        static constexpr const char* GTO = "GTO";  // gto : go to (will perform go to with the actions set in pos, for, spe)
        static constexpr const char* ATR = "ATR";  // atr : auto-release (emergency slow move)
        static constexpr const char* ADR = "ADR";  // adr : auto-release direction (open(1) or close(0) during auto-release)
        static constexpr const char* FOR = "FOR";  // for : force (0-255)
        static constexpr const char* SPE = "SPE";  // spe : speed (0-255)
        static constexpr const char* POS = "POS";  // pos : position (0-255), 0 = open
    };

    struct ReadVariables {
        static constexpr const char* STA = "STA";  // status (0 = is reset, 1 = activating, 3 = active)
        static constexpr const char* PRE = "PRE";  // position request (echo of last commanded position)
        static constexpr const char* OBJ = "OBJ";  // object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
        static constexpr const char* FLT = "FLT";  // fault (0=ok, see manual for errors if not zero)
    };


public:
    static constexpr int DEFAULT_PORT = 63352;
    static constexpr double DEFAULT_TIMEOUT = 30.0 * 1000;
    static constexpr int MIN_POSITION = 0;
    static constexpr int MAX_POSITION = 255;
    static constexpr int MIN_SPEED = 0;
    static constexpr int MAX_SPEED = 255;
    static constexpr int MIN_FORCE = 0;
    static constexpr int MAX_FORCE = 255;

    RobotiqSocket(const std::string& address, const int port = DEFAULT_PORT);
    ~RobotiqSocket();
    
    Status connect(double socket_timeout = DEFAULT_TIMEOUT);
    void disconnect();
    Status activate(const double socket_timeout=DEFAULT_TIMEOUT);

    bool is_connected();
    bool is_active();
    // int get_min_position();
    // int get_max_position();
    int get_open_position();
    int get_closed_position();
    int get_range();
    bool is_open();
    bool is_closed();
    bool is_moving();
    bool is_moving_received(const MoveResult& res);
    int get_current_position();
    int get_current_velocity();
    int get_current_effort();

    bool is_stuck();
    bool stop_movement();
    bool reset_velocity();

    MoveResult move(int position, int speed, int force);
    MoveResult move_and_wait_for_pos(int position, int speed, int force);
};
} // namespace robotiq_driver
