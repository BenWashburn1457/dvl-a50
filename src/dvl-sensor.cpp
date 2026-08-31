/**
 * @file   dvl-sensor.cpp
 *
 * @author Pablo Gutiérrez
 * @date   24/11/2021
 */

#include "dvl_a50/dvl-sensor.hpp"
#include <cerrno>

namespace dvl_sensor {

// How often to retry while disconnected. Also roughly bounds how long attempt_connect()
// can block the executor on a single failed connect() (see Connect()'s timeout_ms below).
static constexpr std::chrono::seconds RECONNECT_PERIOD{3};

DVL_A50::DVL_A50():
Node("dvl_a50_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);

    //Publishers
    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", qos);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", qos);
    dvl_pub_config_status = this->create_publisher<dvl_msgs::msg::ConfigStatus>("dvl/config/status", qos);
    dvl_pub_command_response = this->create_publisher<dvl_msgs::msg::CommandResponse>("dvl/command/response", qos);
    dvl_sub_config_command = this->create_subscription<dvl_msgs::msg::ConfigCommand>("dvl/config/command", qos, std::bind(&DVL_A50::command_subscriber, this, _1));

    this->declare_parameter<std::string>("dvl_ip_address", "192.168.194.95");
    this->declare_parameter<std::string>("dvl_frame_id", "dvl_link");
    this->declare_parameter<std::string>("odom_frame_id", "dvl_odom_ned");
    this->declare_parameter<bool>("startup_disable_acoustics", false);
    this->declare_parameter<std::string>("startup_ntp_address", "");

    dvl_frame_id = this->get_parameter("dvl_frame_id").as_string();
    odom_frame_id = this->get_parameter("odom_frame_id").as_string();
    ip_address = this->get_parameter("dvl_ip_address").as_string();
    disable_acoustics_on_startup_ = this->get_parameter("startup_disable_acoustics").as_bool();
    startup_ntp_address_ = this->get_parameter("startup_ntp_address").as_string();
    RCLCPP_INFO(get_logger(), "IP_ADDRESS: '%s'", ip_address.c_str());

    // Make one attempt right away so a DVL that's already on connects immediately, without
    // waiting for the first reconnect-timer tick.
    attempt_connect();
    timer_reconnect = this->create_wall_timer(RECONNECT_PERIOD, std::bind(&DVL_A50::attempt_connect, this));
    timer_receive = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DVL_A50::handle_receive, this));
}

DVL_A50::~DVL_A50() {
    if (tcpSocket)
    {
        tcpSocket->Close();
        delete tcpSocket;
    }
}

/*
 * Makes one attempt to (re)connect to the DVL. Called once from the constructor and then
 * repeatedly from timer_reconnect, so the node picks up the sensor whenever it actually
 * becomes reachable - whether that's before or long after this node started - and recovers
 * automatically if the connection later drops (see handle_receive()).
 */
void DVL_A50::attempt_connect()
{
    if (connection_state_ == ConnectionState::CONNECTED)
        return;

    // TCPSocket has no "reset" - its fd is dead after Close(), so a fresh instance is needed
    // for each attempt. Deleting a null tcpSocket (first call) is a safe no-op.
    delete tcpSocket;
    tcpSocket = new TCPSocket((char*)ip_address.c_str(), 16171);

    if (tcpSocket->Create() < 0)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Error creating the DVL socket, will keep retrying.");
        return;
    }

    tcpSocket->SetRcvTimeout(400);

    // Named connect_error_msg, not error, to avoid shadowing the public uint8_t error member.
    std::string connect_error_msg;
    int error_code = 0;
    int result = tcpSocket->Connect(2000, connect_error_msg, error_code);

    if (result != 0)
    {
        if (error_code == 114)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Is the DVL powered on? error_code: %d", error_code);
        else if (error_code == 103)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "No route to host, DVL might be booting or unplugged: error_code: %d", error_code);
        else
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "DVL connect failed: %s (error_code: %d)", connect_error_msg.c_str(), error_code);

        tcpSocket->Close();
        return;
    }

    RCLCPP_INFO(get_logger(), "DVL-A50 connected!");
    connection_state_ = ConnectionState::CONNECTED;
    this->ready = 1;
    this->error = 0;
    on_connected();
}

/*
 * Re-applies startup configuration on every successful (re)connect. The physical sensor
 * resets to its own firmware defaults on power-cycle, so this can't just run once at node
 * startup - it needs to run again every time the link comes back up.
 */
void DVL_A50::on_connected()
{
    if (disable_acoustics_on_startup_)
    {
        this->set_json_parameter("acoustic_enabled", "false");
        RCLCPP_INFO(get_logger(), "Acoustic operation disabled on (re)connect.");
    }

    if (!startup_ntp_address_.empty())
    {
        json ntp_command = {
            {"command", "set_time_ntp"},
            {"parameters", {{"ntp_address", startup_ntp_address_}}}
        };
        this->send_parameter_to_sensor(ntp_command);
        RCLCPP_INFO(get_logger(), "Setting NTP address to: '%s'", startup_ntp_address_.c_str());
    }
}


void DVL_A50::handle_receive()
{
    if(connection_state_ != ConnectionState::CONNECTED)
        return;

    char byte = 0;
    std::string str;
    int n;

    while((n = tcpSocket->Receive(&byte)) > 0)
    {
        str += byte;
        if (byte == '\n')
            break;
    }

    // n == 0 means the peer closed the connection; a negative n with an errno other than
    // EAGAIN/EWOULDBLOCK/EINTR (the expected outcome of the 400ms recv timeout when the DVL
    // simply has nothing new to send) means the socket is actually broken. Either way the
    // link is gone - drop back to disconnected so timer_reconnect takes back over.
    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
    {
        RCLCPP_WARN(get_logger(), "Lost connection to the DVL - will attempt to reconnect.");
        tcpSocket->Close();
        connection_state_ = ConnectionState::DISCONNECTED;
        this->ready = 0;
        this->error = 1;
        return;
    }

    // Skip parse if we got nothing or a partial message (e.g. socket timeout mid-stream)
    if (str.empty() || str.back() != '\n')
        return;

    try
    {
        json_data = json::parse(str);

        if (json_data.contains("altitude")) {
            this->publish_vel_trans_report();
        }
        else if (json_data.contains("pitch")) {
            this->publish_dead_reckoning_report();
        }
        else if (json_data.contains("response_to"))
        {
            const std::string resp = json_data["response_to"];
            if (resp == "set_config"
            || resp == "calibrate_gyro"
            || resp == "reset_dead_reckoning"
            || resp == "set_time_ntp"
            || resp == "set_time_manual"
            || resp == "trigger_ping")
                this->publish_command_response();
            else if (resp == "get_config")
                this->publish_config_status();
            else if (resp == "get_time_ntp"
                  || resp == "get_time_status"
                  || resp == "force_sync_ntp")
                this->publish_command_response();
        }
    }
    catch(std::exception& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
}

/*
 * Publish velocity and transductors report
 */
void DVL_A50::publish_vel_trans_report()
{
    // DVL message struct
    dvl_msgs::msg::DVLBeam beam0;
    dvl_msgs::msg::DVLBeam beam1;
    dvl_msgs::msg::DVLBeam beam2;
    dvl_msgs::msg::DVLBeam beam3;
    dvl_msgs::msg::DVL dvl;

    dvl.header.stamp = Node::now();
    dvl.header.frame_id = dvl_frame_id;
		
    dvl.time = double(json_data["time"]);
    dvl.time_of_validity = json_data["time_of_validity"].get<int64_t>();
    dvl.time_of_transmission = json_data["time_of_transmission"].get<int64_t>();

    dvl.velocity.x = double(json_data["vx"]);
    dvl.velocity.y = double(json_data["vy"]);
    dvl.velocity.z = double(json_data["vz"]);
    dvl.fom = double(json_data["fom"]);
    double current_altitude = double(json_data["altitude"]);
    dvl.velocity_valid = json_data["velocity_valid"];
    
    // Removed logic to add old altitude if not valid. Does not report accurately what is coming from the DVL.
    // Logic should be implemented on a case by case basis.
    dvl.altitude = current_altitude;


    dvl.status = json_data["status"];
    dvl.form = json_data["format"];

    // Add covariance from message
    std::vector<double> twistCovariance;

    if (json_data.contains("covariance") && json_data["covariance"].is_array()) {
        const auto& matrix = json_data["covariance"];
        
        for (const auto& row : matrix) {
            if (!row.is_array()) continue;
            for (const auto& value : row) {
                twistCovariance.push_back(value.get<double>());
            }
        }
    }

    dvl.covariance = twistCovariance;
			
    beam0.id = json_data["transducers"][0]["id"];
    beam0.velocity = double(json_data["transducers"][0]["velocity"]);
    beam0.distance = double(json_data["transducers"][0]["distance"]);
    beam0.rssi = double(json_data["transducers"][0]["rssi"]);
    beam0.nsd = double(json_data["transducers"][0]["nsd"]);
    beam0.valid = json_data["transducers"][0]["beam_valid"];
			
    beam1.id = json_data["transducers"][1]["id"];
    beam1.velocity = double(json_data["transducers"][1]["velocity"]);
    beam1.distance = double(json_data["transducers"][1]["distance"]);
    beam1.rssi = double(json_data["transducers"][1]["rssi"]);
    beam1.nsd = double(json_data["transducers"][1]["nsd"]);
    beam1.valid = json_data["transducers"][1]["beam_valid"];
			
    beam2.id = json_data["transducers"][2]["id"];
    beam2.velocity = double(json_data["transducers"][2]["velocity"]);
    beam2.distance = double(json_data["transducers"][2]["distance"]);
    beam2.rssi = double(json_data["transducers"][2]["rssi"]);
    beam2.nsd = double(json_data["transducers"][2]["nsd"]);
    beam2.valid = json_data["transducers"][2]["beam_valid"];
			
    beam3.id = json_data["transducers"][3]["id"];
    beam3.velocity = double(json_data["transducers"][3]["velocity"]);
    beam3.distance = double(json_data["transducers"][3]["distance"]);
    beam3.rssi = double(json_data["transducers"][3]["rssi"]);
    beam3.nsd = double(json_data["transducers"][3]["nsd"]);
    beam3.valid = json_data["transducers"][3]["beam_valid"];
		    
    dvl.beams = {beam0, beam1, beam2, beam3};
    dvl_pub_report->publish(dvl);
}

/*
 * Publish dead reckoning
 */
void DVL_A50::publish_dead_reckoning_report()
{
    dvl_msgs::msg::DVLDR DVLDeadReckoning;
    //std::cout << std::setw(4) << json_data << std::endl;
    DVLDeadReckoning.header.stamp = Node::now();
    DVLDeadReckoning.header.frame_id = odom_frame_id;
    DVLDeadReckoning.child_frame_id = dvl_frame_id;
    DVLDeadReckoning.time = double(json_data["ts"]);
    DVLDeadReckoning.position.x = double(json_data["x"]);
    DVLDeadReckoning.position.y = double(json_data["y"]);
    DVLDeadReckoning.position.z = double(json_data["z"]);
    DVLDeadReckoning.pos_std = double(json_data["std"]);
    DVLDeadReckoning.roll = double(json_data["roll"]);
    DVLDeadReckoning.pitch = double(json_data["pitch"]);
    DVLDeadReckoning.yaw = double(json_data["yaw"]);
    DVLDeadReckoning.type = json_data["type"];
    DVLDeadReckoning.status = json_data["status"];
    DVLDeadReckoning.format = json_data["format"];
    dvl_pub_pos->publish(DVLDeadReckoning);
}

/*
 * Publish the command response
 */
void DVL_A50::publish_command_response()
{
    dvl_msgs::msg::CommandResponse command_resp;
    command_resp.response_to = json_data["response_to"];
    command_resp.success = json_data["success"];
    command_resp.error_message = json_data["error_message"];
    command_resp.result = 0;
    command_resp.format = json_data["format"];
    command_resp.type = json_data["type"];

    const std::string resp = command_resp.response_to;
    if (json_data.contains("result") && json_data["result"].is_object()) {
        const auto& result = json_data["result"];
        auto& ts = command_resp.time_status;

        if (resp == "get_time_ntp") {
            ts.ntp_address = result["ntp_address"];
        } else if (resp == "get_time_status") {
            ts.system_time = result["system_time"];
            ts.ntp_synced = result["ntp_synced"];
            ts.ntp_synced_to = result["ntp_synced_to"];
            ts.ntp_seconds_since_last_sync = result["ntp_seconds_since_last_sync"].is_null()
                ? -1.0 : result["ntp_seconds_since_last_sync"].get<double>();
        } else if (resp == "force_sync_ntp" && result.contains("status") && result["status"].is_object()) {
            const auto& status = result["status"];
            ts.system_time = status["system_time"];
            ts.ntp_synced = status["ntp_synced"];
            ts.ntp_synced_to = status["ntp_synced_to"];
            ts.ntp_seconds_since_last_sync = status["ntp_seconds_since_last_sync"].is_null()
                ? -1.0 : status["ntp_seconds_since_last_sync"].get<double>();
        }
    }

    dvl_pub_command_response->publish(command_resp);
}

/*
 * Publish the sensor current configuration
 */
void DVL_A50::publish_config_status()
{
    dvl_msgs::msg::ConfigStatus status_msg;
    status_msg.response_to = json_data["response_to"];
    status_msg.success = json_data["success"];
    status_msg.error_message = json_data["error_message"];
    status_msg.speed_of_sound = json_data["result"]["speed_of_sound"];
    status_msg.acoustic_enabled = json_data["result"]["acoustic_enabled"];
    status_msg.dark_mode_enabled = json_data["result"]["dark_mode_enabled"];
    status_msg.periodic_cycling_enabled = json_data["result"]["periodic_cycling_enabled"];
    status_msg.mounting_rotation_offset = json_data["result"]["mounting_rotation_offset"];
    status_msg.range_mode = json_data["result"]["range_mode"];
    status_msg.format = json_data["format"];
    status_msg.type = json_data["type"];
    dvl_pub_config_status->publish(status_msg);
}


void DVL_A50::command_subscriber(const dvl_msgs::msg::ConfigCommand::SharedPtr msg)
{
    if(msg->command == "set_config")
        this->set_json_parameter(msg->parameter_name, msg->parameter_value);
    else if(msg->command == "get_config")
    {
        json command = {
            {"command", "get_config"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "calibrate_gyro")
    {
        json command = {
            {"command", "calibrate_gyro"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "reset_dead_reckoning")
    {
        json command = {
            {"command", "reset_dead_reckoning"}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "set_time_ntp")
    {
        json command = {
            {"command", "set_time_ntp"},
            {"parameters", {{"ntp_address", msg->parameter_value}}}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "get_time_ntp")
    {
        json command = {{"command", "get_time_ntp"}};
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "set_time_manual")
    {
        json command = {
            {"command", "set_time_manual"},
            {"parameters", {{"now", msg->parameter_value}}}
        };
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "get_time_status")
    {
        json command = {{"command", "get_time_status"}};
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "trigger_ping")
    {
        json command = {{"command", "trigger_ping"}};
        this->send_parameter_to_sensor(command);
    }
    else if(msg->command == "force_sync_ntp")
    {
        try
        {
            int timeout = std::stoi(msg->parameter_value);
            json command = {
                {"command", "force_sync_ntp"},
                {"parameters", {{"timeout_seconds", timeout}}}
            };
            this->send_parameter_to_sensor(command);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "force_sync_ntp requires integer timeout_seconds: %s", e.what());
        }
    }

}

DVL_Parameters DVL_A50::resolveParameter(std::string param)
{
	if(param == "speed_of_sound")
	    return speed_of_sound;
	else if(param == "acoustic_enabled")
	    return acoustic_enabled;
	else if(param == "dark_mode_enabled")
	    return dark_mode_enabled;
	else if(param == "periodic_cycling_enabled")
	    return periodic_cycling_enabled;
	else if(param == "mounting_rotation_offset")
	    return mounting_rotation_offset;
	else if(param == "range_mode")
	    return range_mode;

	return invalid_param;
}

/*
 * Create JSON command message to set parameter in the DVL sensor
 */
void DVL_A50::set_json_parameter(const std::string name, const std::string value)
{
    json message;
    message["command"] = "set_config";

    switch (resolveParameter(name))
    {
        case speed_of_sound:
            try
            {
                message["parameters"]["speed_of_sound"] = (int)std::stoi(value);
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case acoustic_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["acoustic_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case dark_mode_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["dark_mode_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case periodic_cycling_enabled:
            try
            {
                bool data;
                std::istringstream(value) >> std::boolalpha >> data;
                message["parameters"]["periodic_cycling_enabled"] = data;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case mounting_rotation_offset:
            try
            {
                message["parameters"]["mounting_rotation_offset"] = (double)std::stod(value);
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        case range_mode:
            try
            {
                message["parameters"]["range_mode"] = value;
                this->send_parameter_to_sensor(message);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "Invalid data type! error: %s", e.what());
            }
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Invalid parameter!");
            break;
    }

}

/*
 * Send parameter to the sensor using the JSON command
 */
void DVL_A50::send_parameter_to_sensor(const json &message)
{
    std::string str = message.dump();
    char* c = &*str.begin();
    tcpSocket->Send(c);
    usleep(100000); // 100ms between commands to avoid flooding the sensor
}

}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dvl_sensor::DVL_A50>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
