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

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <cstring>  // for memcpy
#include <iostream>
#include <vector>

#include "oy_motor_constants.hpp"

namespace openarm::oy_motor {
// Forward declarations
class Motor;

struct ParamResult {
    int rid;
    double value;
    bool valid;
};

struct StateResult {
    double position;
    double velocity;
    double torque;
    int t_mos;
    int t_rotor;
    bool valid;
};

struct CANPacket {
    uint32_t send_can_id;
    std::vector<uint8_t> data;
};

struct MITParam {
    double kp;
    double kd;
    double q;
    double dq;
    double tau;
};

struct PosVelParam {
    double q;
    double dq;
};

class CanPacketEncoder {
public:
    static CANPacket create_reboot_command(const Motor& motor);
    static CANPacket create_enable_command(const Motor& motor);
    static CANPacket create_disable_command(const Motor& motor);
    static CANPacket create_set_zero_command(const Motor& motor);
    static CANPacket create_mit_control_command(const Motor& motor, const MITParam& mit_param);
    static CANPacket create_posvel_control_command(const Motor& motor,
                                                   const PosVelParam& posvel_param);
    // Custom-protocol query: pass a command code (e.g., 0xA4, 0xAE, 0xF1, 0xF0, ...)
    static CANPacket create_query_param_command(const Motor& motor, int command_code);
    static CANPacket create_refresh_command(const Motor& motor);

    // Custom protocol control surface
    static CANPacket create_clear_fault_command(const Motor& motor);
    static CANPacket create_brake_command(const Motor& motor, uint8_t op);
    //create iq current control command
    static CANPacket create_iq_control_command(const Motor& motor, double iq_current_a);
    static CANPacket create_speed_control_command(const Motor& motor, double speed_rpm);
    static CANPacket create_abs_pos_control_command_rad(const Motor& motor, double position_rad);
    static CANPacket create_rel_pos_control_command_rad(const Motor& motor, double delta_rad);
    static CANPacket create_shortest_home_command(const Motor& motor);

    // MIT limits (0xF0): query or set (persisted on drive board)
    static CANPacket create_query_mit_limits_command(const Motor& motor);
    static CANPacket create_set_mit_limits_command(const Motor& motor, double pos_max_rad,
                                                   double vel_max_rad_s, double t_max_nm);

private:
    static std::vector<uint8_t> pack_mit_control_data(const Motor& motor, const MITParam& mit_param);
    static std::vector<uint8_t> pack_abs_pos_control_data(int32_t counts);
    static std::vector<uint8_t> pack_iq_control_data(int32_t iq_milliamp);
    static std::vector<uint8_t> pack_speed_control_data(int32_t rpm_0p01);
    static std::vector<uint8_t> pack_rel_pos_control_data(int32_t delta_counts);
    static std::vector<uint8_t> pack_brake_control_data(uint8_t op);
    static std::vector<uint8_t> pack_mit_limits_set_data(double pos_max_rad, double vel_max_rad_s,
                                                         double t_max_nm);
    static std::vector<uint8_t> pack_reboot_data();   
                                                  
    static std::vector<uint8_t> pack_command_only(uint8_t cmd);

    static double limit_min_max(double x, double min, double max);
    static uint16_t double_to_uint(double x, double x_min, double x_max, int bits);
    static std::array<uint8_t, 4> float_to_uint8s(float value);
    static void append_int32_le(std::vector<uint8_t>& out, int32_t value);
};

class CanPacketDecoder {
public:
    // Parses MIT-like packed state frames (e.g., 0xF1 response payload)
    static StateResult parse_motor_state_data(const Motor& motor, const std::vector<uint8_t>& data);

    // Parses 0xA4 response and converts into canonical units (rad/rad/s/Nm/°C).
    static StateResult parse_a4_state_data(Motor& motor, const std::vector<uint8_t>& data);
    // Returns zero or more decoded scalar fields keyed by (command_code<<8 | field_index).
    static std::vector<ParamResult> parse_motor_param_data(Motor& motor, const std::vector<uint8_t>& data);

private:
    static double uint_to_double(uint16_t x, double min, double max, int bits);
    static float uint8s_to_float(const std::array<uint8_t, 4>& bytes);
    static uint32_t uint8s_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);
    static bool is_in_ranges(int number);

    static int32_t int32_from_le(const std::vector<uint8_t>& data, size_t offset);
    static int16_t int16_from_le(const std::vector<uint8_t>& data, size_t offset);
    static uint16_t uint16_from_le(const std::vector<uint8_t>& data, size_t offset);
};

}  // namespace openarm::oy_motor
