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

union OctoplaneVtolAirframeIn3dSim {
    struct {
        double motor_front_right_rpm;
        double motor_rear_left_rpm;
        double motor_front_left_rpm;
        double motor_rear_right_rpm;

        double left_aileron_deg;
        double right_aileron_deg;
        double elevator_deg;
        double rudder_deg;

        double pusher_motor_rpm;

        void print()
        {
            std::cout << "Actuators: ["
                      << motor_front_right_rpm << ", "
                      << motor_rear_left_rpm << ", "
                      << motor_front_left_rpm << ", "
                      << motor_rear_right_rpm << "], ("
                      << left_aileron_deg << ", "
                      << right_aileron_deg << ", "
                      << elevator_deg << ", "
                      << rudder_deg << "), throttle="
                      << pusher_motor_rpm << "." << std::endl;
        }
    } controls;

    double values[sizeof(controls) / sizeof(double)];
};

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
        _timer = std::make_unique<boost::asio::steady_timer>(_client.get_io_service(), std::chrono::milliseconds(40));  // 25 Hz rate
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
        // OctoplaneVtolAirframeIn3dSim airframe = {
        //     .controls = {
        //         .motor_front_right_rpm = _sim->getActuatorState(0) * (60 / 2 / 3.14),
        //         .motor_rear_left_rpm = _sim->getActuatorState(1) * (60 / 2 / 3.14),
        //         .motor_front_left_rpm = _sim->getActuatorState(2) * (60 / 2 / 3.14),
        //         .motor_rear_right_rpm = _sim->getActuatorState(3) * (60 / 2 / 3.14),

        //         .left_aileron_deg = 1.5 * _sim->getActuatorState(5) * (180 / 3.14),
        //         .right_aileron_deg = 1.5 * _sim->getActuatorState(6) * (180 / 3.14),
        //         .elevator_deg = 1.5 * _sim->getActuatorState(7) * (180 / 3.14),
        //         .rudder_deg = +50,

        //         .pusher_motor_rpm = _sim->getActuatorState(4) * (60 / 2 / 3.14),
        //     }
        // };

        OctoplaneVtolAirframeIn3dSim airframe = {
            .controls = {
                .motor_front_right_rpm = 0.0,
                .motor_rear_left_rpm = 0.0,
                .motor_front_left_rpm = 0.0,
                .motor_rear_right_rpm = 0.0,

                .left_aileron_deg = 1.5 * _sim->getActuatorState(5) * (180 / 3.14),
                .right_aileron_deg = 1.5 * _sim->getActuatorState(6) * (180 / 3.14),
                .elevator_deg = 1.5 * _sim->getActuatorState(7) * (180 / 3.14),
                .rudder_deg = 1.5 * _sim->getActuatorState(2) * (180 / 3.14),

                .pusher_motor_rpm = _sim->getActuatorState(4) * (60 / 2 / 3.14),
            }
        };

        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "base_link"}}},
            {"buttons", {1, 0, 0, 1}}
        };
        msg["axes"] = std::vector<double>(std::begin(airframe.values), std::end(airframe.values));

        airframe.controls.print();
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
