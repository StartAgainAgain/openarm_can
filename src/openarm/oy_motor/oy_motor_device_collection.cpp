// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <linux/can.h>
#include <linux/can/raw.h>

#include <iostream>
#include <openarm/oy_motor/oy_motor.hpp>
#include <openarm/oy_motor/oy_motor_device.hpp>
#include <openarm/oy_motor/oy_motor_device_collection.hpp>

namespace openarm::oy_motor {

OYDeviceCollection::OYDeviceCollection(canbus::CANSocket& can_socket)
    : can_socket_(can_socket),
      can_packet_encoder_(std::make_unique<CanPacketEncoder>()),
      can_packet_decoder_(std::make_unique<CanPacketDecoder>()),
      device_collection_(std::make_unique<canbus::CANDeviceCollection>(can_socket_)) {}

void OYDeviceCollection::enable_all() {
    for (auto oy_device : get_oy_devices()) {
        auto& motor = oy_device->get_motor();
        CANPacket enable_packet = CanPacketEncoder::create_enable_command(motor);
        send_command_to_device(oy_device, enable_packet);
    }
}

void OYDeviceCollection::disable_all() {
    for (auto oy_device : get_oy_devices()) {
        CANPacket disable_packet = CanPacketEncoder::create_disable_command(oy_device->get_motor());
        send_command_to_device(oy_device, disable_packet);
    }
}

void OYDeviceCollection::set_zero(int i) {
    auto oy_device = get_oy_devices().at(i);
    auto zero_packet = CanPacketEncoder::create_set_zero_command(oy_device->get_motor());
    send_command_to_device(oy_device, zero_packet);
}

void OYDeviceCollection::set_zero_all() {
    for (auto oy_device : get_oy_devices()) {
        CANPacket zero_packet = CanPacketEncoder::create_set_zero_command(oy_device->get_motor());
        send_command_to_device(oy_device, zero_packet);
    }
}

void OYDeviceCollection::refresh_one(int i) {
    auto oy_device = get_oy_devices().at(i);
    auto& motor = oy_device->get_motor();
    CANPacket refresh_packet = CanPacketEncoder::create_refresh_command(motor);
    send_command_to_device(oy_device, refresh_packet);
}

void OYDeviceCollection::refresh_all() {
    for (auto oy_device : get_oy_devices()) {
        auto& motor = oy_device->get_motor();
        CANPacket refresh_packet = CanPacketEncoder::create_refresh_command(motor);
        send_command_to_device(oy_device, refresh_packet);
    }
}

void OYDeviceCollection::set_callback_mode_all(CallbackMode callback_mode) {
    for (auto oy_device : get_oy_devices()) {
        oy_device->set_callback_mode(callback_mode);
    }
}

void OYDeviceCollection::query_param_one(int i, int RID) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket param_query =
        CanPacketEncoder::create_query_param_command(oy_device->get_motor(), RID);
    send_command_to_device(oy_device, param_query);
}

void OYDeviceCollection::query_param_all(int RID) {
    for (auto oy_device : get_oy_devices()) {
        CANPacket param_query =
            CanPacketEncoder::create_query_param_command(oy_device->get_motor(), RID);
        send_command_to_device(oy_device, param_query);
    }
}

void OYDeviceCollection::send_command_to_device(std::shared_ptr<OYCANDevice> oy_device,
                                                const CANPacket& packet) {
    if (can_socket_.is_canfd_enabled()) {
        canfd_frame frame = oy_device->create_canfd_frame(packet.send_can_id, packet.data);
        can_socket_.write_canfd_frame(frame);
    } else {
        can_frame frame = oy_device->create_can_frame(packet.send_can_id, packet.data);
        can_socket_.write_can_frame(frame);
    }
}

void OYDeviceCollection::mit_control_one(int i, const MITParam& mit_param) {
    CANPacket mit_cmd =
        CanPacketEncoder::create_mit_control_command(get_oy_devices()[i]->get_motor(), mit_param);
    send_command_to_device(get_oy_devices()[i], mit_cmd);
}

void OYDeviceCollection::mit_control_all(const std::vector<MITParam>& mit_params) {
    for (size_t i = 0; i < mit_params.size(); i++) {
        mit_control_one(i, mit_params[i]);
    }
}

void OYDeviceCollection::posvel_control_one(int i, const PosVelParam& posvel_param) {
    CANPacket posvel_cmd = CanPacketEncoder::create_posvel_control_command(
        get_oy_devices()[i]->get_motor(), posvel_param);
    send_command_to_device(get_oy_devices()[i], posvel_cmd);
}

void OYDeviceCollection::posvel_control_all(const std::vector<PosVelParam>& posvel_params) {
    for (size_t i = 0; i < posvel_params.size(); i++) {
        posvel_control_one(i, posvel_params[i]);
    }
}

void OYDeviceCollection::clear_fault_one(int i) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_clear_fault_command(oy_device->get_motor());
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::clear_fault_all() {
    for (auto oy_device : get_oy_devices()) {
        CANPacket cmd = CanPacketEncoder::create_clear_fault_command(oy_device->get_motor());
        send_command_to_device(oy_device, cmd);
    }
}

void OYDeviceCollection::brake_one(int i, uint8_t op) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_brake_command(oy_device->get_motor(), op);
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::brake_all(uint8_t op) {
    for (auto oy_device : get_oy_devices()) {
        CANPacket cmd = CanPacketEncoder::create_brake_command(oy_device->get_motor(), op);
        send_command_to_device(oy_device, cmd);
    }
}

void OYDeviceCollection::iq_control_one(int i, double iq_current_a) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_iq_control_command(oy_device->get_motor(), iq_current_a);
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::iq_control_all(const std::vector<double>& iq_currents_a) {
    for (size_t i = 0; i < iq_currents_a.size(); i++) {
        iq_control_one(static_cast<int>(i), iq_currents_a[i]);
    }
}

void OYDeviceCollection::speed_control_one(int i, double speed_rpm) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_speed_control_command(oy_device->get_motor(), speed_rpm);
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::speed_control_all(const std::vector<double>& speeds_rpm) {
    for (size_t i = 0; i < speeds_rpm.size(); i++) {
        speed_control_one(static_cast<int>(i), speeds_rpm[i]);
    }
}

void OYDeviceCollection::abs_pos_control_one_rad(int i, double position_rad) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd =
        CanPacketEncoder::create_abs_pos_control_command_rad(oy_device->get_motor(), position_rad);
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::abs_pos_control_all_rad(const std::vector<double>& positions_rad) {
    for (size_t i = 0; i < positions_rad.size(); i++) {
        abs_pos_control_one_rad(static_cast<int>(i), positions_rad[i]);
    }
}

void OYDeviceCollection::rel_pos_control_one_rad(int i, double delta_rad) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd =
        CanPacketEncoder::create_rel_pos_control_command_rad(oy_device->get_motor(), delta_rad);
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::rel_pos_control_all_rad(const std::vector<double>& deltas_rad) {
    for (size_t i = 0; i < deltas_rad.size(); i++) {
        rel_pos_control_one_rad(static_cast<int>(i), deltas_rad[i]);
    }
}

void OYDeviceCollection::shortest_home_one(int i) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_shortest_home_command(oy_device->get_motor());
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::shortest_home_all() {
    for (auto oy_device : get_oy_devices()) {
        CANPacket cmd = CanPacketEncoder::create_shortest_home_command(oy_device->get_motor());
        send_command_to_device(oy_device, cmd);
    }
}

void OYDeviceCollection::query_mit_limits_one(int i) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_query_mit_limits_command(oy_device->get_motor());
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::query_mit_limits_all() {
    for (auto oy_device : get_oy_devices()) {
        CANPacket cmd = CanPacketEncoder::create_query_mit_limits_command(oy_device->get_motor());
        send_command_to_device(oy_device, cmd);
    }
}

void OYDeviceCollection::set_mit_limits_one(int i, double pos_max_rad, double vel_max_rad_s,
                                           double t_max_nm) {
    auto oy_device = get_oy_devices().at(i);
    CANPacket cmd = CanPacketEncoder::create_set_mit_limits_command(oy_device->get_motor(),
                                                                    pos_max_rad, vel_max_rad_s,
                                                                    t_max_nm);
    send_command_to_device(oy_device, cmd);
}

void OYDeviceCollection::set_mit_limits_all(double pos_max_rad, double vel_max_rad_s,
                                           double t_max_nm) {
    for (auto oy_device : get_oy_devices()) {
        CANPacket cmd = CanPacketEncoder::create_set_mit_limits_command(oy_device->get_motor(),
                                                                        pos_max_rad, vel_max_rad_s,
                                                                        t_max_nm);
        send_command_to_device(oy_device, cmd);
    }
}

std::vector<Motor> OYDeviceCollection::get_motors() const {
    std::vector<Motor> motors;
    for (auto oy_device : get_oy_devices()) {
        motors.push_back(oy_device->get_motor());
    }
    return motors;
}

Motor OYDeviceCollection::get_motor(int i) const { return get_oy_devices().at(i)->get_motor(); }

std::vector<std::shared_ptr<OYCANDevice>> OYDeviceCollection::get_oy_devices() const {
    std::vector<std::shared_ptr<OYCANDevice>> oy_devices;
    for (const auto& [id, device] : device_collection_->get_devices()) {
        auto oy_device = std::dynamic_pointer_cast<OYCANDevice>(device);
        if (oy_device) {
            oy_devices.push_back(oy_device);
        }
    }
    return oy_devices;
}

}  // namespace openarm::oy_motor
