/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */
#ifndef ROS_INTERFACE_ROS_INTERFACE_HPP
#define ROS_INTERFACE_ROS_INTERFACE_HPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>
#include <memory>
#include "UavDynamics/sim.hpp"

class RosPublisher {
public:
    RosPublisher(std::shared_ptr<UavDynamics::Sim> sim, const std::string& uri = "ws://localhost:9090") :
        _client(),
        _server_uri(uri),
        _timer(nullptr),
        _sim(sim)
    {
        _client.init_asio();

        // Disable all logging
        _client.clear_access_channels(websocketpp::log::alevel::all);
        _client.clear_error_channels(websocketpp::log::elevel::all);

        // Set WebSocket handlers
        _client.set_open_handler([this](websocketpp::connection_hdl hdl) {
            this->on_open(hdl);
        });
        _client.set_close_handler([this](websocketpp::connection_hdl hdl) {
            std::cout << "Connection closed." << std::endl;
        });
        _client.set_fail_handler([this](websocketpp::connection_hdl hdl) {
            std::cerr << "Connection failed." << std::endl;
        });

        websocketpp::lib::error_code ec;
        auto con = _client.get_connection(_server_uri, ec);
        if (ec) {
            std::cerr << "Connection failed: " << ec.message() << std::endl;
            return;
        }
        _client.connect(con);
    }

    void run() {
        _client.run();
    }

private:
    websocketpp::client<websocketpp::config::asio_client> _client;
    std::string _server_uri;
    websocketpp::connection_hdl connection_;
    std::unique_ptr<boost::asio::steady_timer> _timer;
    std::shared_ptr<UavDynamics::Sim> _sim;

    void on_open(websocketpp::connection_hdl hdl) {
        connection_ = hdl;
        std::cout << "Connected to rosbridge server at " << _server_uri << std::endl;

        // Advertise the topics
        advertise_topic("/sim/attitude", "geometry_msgs/QuaternionStamped");
        advertise_topic("/sim/actuators", "sensor_msgs/Joy");
        advertise_topic("/sim/gps_position", "sensor_msgs/NavSatFix");

        // Start periodic publishing
        start_timer();
    }

    void start_timer() {
        _timer = std::make_unique<boost::asio::steady_timer>(_client.get_io_service(), std::chrono::milliseconds(100));  // 10 Hz rate
        _timer->async_wait([this](const boost::system::error_code& ec) {
            if (!ec) {
                this->publish_messages();
                this->start_timer();
            }
        });
    }

    void advertise_topic(const std::string& topic, const std::string& type) {
        nlohmann::json advertise_msg = {
            {"op", "advertise"},
            {"topic", topic},
            {"type", type}
        };
        try {
            _client.send(connection_, advertise_msg.dump(), websocketpp::frame::opcode::text);
            std::cout << "Advertised topic: " << topic << " with type: " << type << std::endl;
        } catch (const websocketpp::exception& e) {
            std::cerr << "Error advertising topic: " << e.what() << std::endl;
        }
    }

    void publish_messages() {
        publish_attitude();
        publish_actuators();
        publish_gps_position();
    }

    void publish_attitude() {
        auto attitude = _sim->attitudeFluEnu();
        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "base_link"}}},
            {"quaternion", {{"x", attitude.x()}, {"y", attitude.y()}, {"z", attitude.z()}, {"w", attitude.w()}}}
        };
        publish_message("/sim/attitude", msg);
    }

    void publish_actuators() {
        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "base_link"}}},
            {"axes", {10, 50, 100, 200, 30, -30, -30, +1, 400}},  // Example axes values
            {"buttons", {1, 0, 0, 1}}        // Example button values
        };
        publish_message("/sim/actuators", msg);
    }

    void publish_gps_position() {
        auto gps = _sim->geoPosition();
        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "gps"}}},
            {"latitude", gps[0]},
            {"longitude", gps[1]},
            {"altitude", gps[2]},
            {"position_covariance", {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1}},
            {"position_covariance_type", 2}
        };
        publish_message("/sim/gps_position", msg);
    }

    void publish_message(const std::string& topic, const nlohmann::json& message) {
        nlohmann::json publish_msg = {
            {"op", "publish"},
            {"topic", topic},
            {"msg", message}
        };
        try {
            _client.send(connection_, publish_msg.dump(), websocketpp::frame::opcode::text);
        } catch (const websocketpp::exception& e) {
            std::cerr << "Error sending message: " << e.what() << std::endl;
        }
    }
};

#endif  // ROS_INTERFACE_ROS_INTERFACE_HPP
