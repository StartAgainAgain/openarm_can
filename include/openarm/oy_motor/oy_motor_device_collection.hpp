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

#pragma once

#include <memory>
#include <vector>

#include "../canbus/can_device_collection.hpp"
#include "oy_motor_constants.hpp"
#include "oy_motor.hpp"
#include "oy_motor_control.hpp"
#include "oy_motor_device.hpp"

namespace openarm::oy_motor {

class DMDeviceCollection {
public:
    DMDeviceCollection(canbus::CANSocket& can_socket);
    virtual ~DMDeviceCollection() = default;

    // Common motor operations
    void enable_all();
    void disable_all();
    void set_callback_mode_all(CallbackMode callback_mode);

    // Flash new zero position
    void set_zero(int i);
    void set_zero_all();

    // Refresh operations (for individual motors)
    void refresh_one(int i);
    void refresh_all();

    // Query parameter operations
    // NOTE: Protocol uses command codes (0xA0/0xA4/0xAE/0xF0/0xF1...).
    // Parameter name kept for compatibility.
    void query_param_one(int i, int command_code);
    void query_param_all(int command_code);

    // MIT control operations
    void mit_control_one(int i, const MITParam& mit_param);
    void mit_control_all(const std::vector<MITParam>& mit_params);

    // PosVel control operation
    void posvel_control_one(int i, const PosVelParam& posvel_param);
    void posvel_control_all(const std::vector<PosVelParam>& posvel_params);

    // Custom protocol control surface
    void clear_fault_one(int i);
    void clear_fault_all();

    void brake_one(int i, uint8_t op);
    void brake_all(uint8_t op);

    void iq_control_one(int i, double iq_current_a);
    void iq_control_all(const std::vector<double>& iq_currents_a);

    void speed_control_one(int i, double speed_rpm);
    void speed_control_all(const std::vector<double>& speeds_rpm);

    void abs_pos_control_one_rad(int i, double position_rad);
    void abs_pos_control_all_rad(const std::vector<double>& positions_rad);

    void rel_pos_control_one_rad(int i, double delta_rad);
    void rel_pos_control_all_rad(const std::vector<double>& deltas_rad);

    void shortest_home_one(int i);
    void shortest_home_all();

    void query_mit_limits_one(int i);
    void query_mit_limits_all();
    void set_mit_limits_one(int i, double pos_max_rad, double vel_max_rad_s, double t_max_nm);
    void set_mit_limits_all(double pos_max_rad, double vel_max_rad_s, double t_max_nm);

    // Device collection access
    std::vector<Motor> get_motors() const;
    Motor get_motor(int i) const;
    canbus::CANDeviceCollection& get_device_collection() { return *device_collection_; }

protected:
    canbus::CANSocket& can_socket_;
    std::unique_ptr<CanPacketEncoder> can_packet_encoder_;
    std::unique_ptr<CanPacketDecoder> can_packet_decoder_;
    std::unique_ptr<canbus::CANDeviceCollection> device_collection_;

    // Helper methods for subclasses
    void send_command_to_device(std::shared_ptr<DMCANDevice> dm_device, const CANPacket& packet);
    std::vector<std::shared_ptr<DMCANDevice>> get_dm_devices() const;
};
}  // namespace openarm::oy_motor
