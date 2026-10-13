#ifndef DVL_A50_HPP
#define DVL_A50_HPP

/**
 * dvl-sensor.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       24/11/2021
 */

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "dvl_a50/tcpsocket.hpp"

#include <string>
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvl_beam.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "dvl_msgs/msg/config_command.hpp"
#include "dvl_msgs/msg/command_response.hpp"
#include "dvl_msgs/msg/config_status.hpp"
#include "dvl_msgs/msg/time_status.hpp"
#include "std_msgs/msg/bool.hpp"

#include "dvl_a50/json/single_include/nlohmann/json.hpp"
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;
using nlohmann::json;


namespace dvl_sensor {

enum DVL_Parameters {
    speed_of_sound,
    acoustic_enabled,
    dark_mode_enabled,
    periodic_cycling_enabled,
    mounting_rotation_offset,
    range_mode,
    invalid_param
};

enum class ConnectionState {
    DISCONNECTED,
    CONNECTED
};

class DVL_A50: public rclcpp::Node
{
public:

    // Mirrors connection_state_ for anything outside this class that wants to know DVL
    // connectivity (e.g. diagnostics) without depending on ConnectionState directly.
    uint8_t ready = 0;
    uint8_t error = 0;

    DVL_A50();
    ~DVL_A50();

private:
    ConnectionState connection_state_ = ConnectionState::DISCONNECTED;
    string delimiter = ",";
    std::string ip_address;
    std::string dvl_frame_id;
    std::string odom_frame_id;
    bool disable_acoustics_on_startup_ = false;
    std::string startup_ntp_address_;
    TCPSocket *tcpSocket = nullptr;
    json json_data;

    DVL_Parameters resolveParameter(std::string param);

    rclcpp::TimerBase::SharedPtr timer_receive;
    rclcpp::TimerBase::SharedPtr timer_reconnect;
    rclcpp::TimerBase::SharedPtr timer_connected;
    rclcpp::Publisher<dvl_msgs::msg::DVL>::SharedPtr dvl_pub_report;
    rclcpp::Publisher<dvl_msgs::msg::DVLDR>::SharedPtr dvl_pub_pos;
    rclcpp::Publisher<dvl_msgs::msg::CommandResponse>::SharedPtr dvl_pub_command_response;
    rclcpp::Publisher<dvl_msgs::msg::ConfigStatus>::SharedPtr dvl_pub_config_status;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dvl_pub_connected;
    rclcpp::Subscription<dvl_msgs::msg::ConfigCommand>::SharedPtr dvl_sub_config_command;

    // Makes one non-blocking-ish attempt to (re)connect to the DVL. Safe to call whether
    // currently connected or not - a no-op while already connected. Bound to timer_reconnect
    // so it keeps retrying indefinitely whenever the sensor isn't reachable yet.
    void attempt_connect();
    // Re-applies startup configuration (acoustics/NTP) every time a connection is
    // established, since power-cycling the physical sensor resets its firmware config too.
    void on_connected();
    // Publishes the TCP connection state on dvl/connected. Runs on a timer (not just on
    // state changes) so subscribers can also detect this node itself going down.
    void publish_connected();

    void handle_receive();
    //Publish velocity and transducer report
    void publish_vel_trans_report();
    void publish_dead_reckoning_report();
    void publish_config_status();
    void publish_command_response();

    void command_subscriber(const dvl_msgs::msg::ConfigCommand::SharedPtr msg);
    void set_json_parameter(const std::string name, const std::string value);
    void send_parameter_to_sensor(const json &message);

};

} // namespace
#endif //DVL_A50_HPP
