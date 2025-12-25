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

#include <cmath>
#include <cstring>
#include <openarm/oy_motor/oy_motor.hpp>
#include <openarm/oy_motor/oy_motor_constants.hpp>
#include <openarm/oy_motor/oy_motor_control.hpp>

#include <algorithm>

namespace openarm::oy_motor {

namespace {
constexpr uint32_t MIT_STDID_MASK = 0x400;

// Temp-param key helper: (command_code<<8 | field_index)
constexpr int make_key(uint8_t cmd, uint8_t field) {
    return (static_cast<int>(cmd) << 8) | static_cast<int>(field);
}
}  // namespace

// Command creation methods (data content per 自定义CAN通信协议_V3.07b0)
CANPacket CanPacketEncoder::create_enable_command(const Motor& motor) {
    // Protocol does not define a dedicated "enable" command.
    // Practical mapping: send torque/current control (0xC0) with 0A to exit 0xCF free state.
    return {motor.get_send_can_id(), pack_iq_control_data(0)};
}

CANPacket CanPacketEncoder::create_disable_command(const Motor& motor) {
    // 0xCF: close motor output (free state)
    return {motor.get_send_can_id(), pack_command_only(0xCF)};
}

CANPacket CanPacketEncoder::create_set_zero_command(const Motor& motor) {
    // 0xB1: set current position as origin
    return {motor.get_send_can_id(), pack_command_only(0xB1)};
}

CANPacket CanPacketEncoder::create_mit_control_command(const Motor& motor,
                                                       const MITParam& mit_param) {
    // MIT control: no command byte; StdID Bit[10]=1 => OR with 0x400
    return {motor.get_send_can_id() | MIT_STDID_MASK, pack_mit_control_data(motor, mit_param)};
}

CANPacket CanPacketEncoder::create_posvel_control_command(const Motor& motor,
                                                          const PosVelParam& posvel_param) {
    // Map to absolute position control (0xC2) using Count (1rev=16384).
    // NOTE: posvel_param.dq is ignored by this protocol mapping.
    double counts_d = posvel_param.q * (16384.0 / (2.0 * M_PI));
    int32_t counts = static_cast<int32_t>(std::llround(counts_d));
    return {motor.get_send_can_id(), pack_abs_pos_control_data(counts)};
}

CANPacket CanPacketEncoder::create_query_param_command(const Motor& motor, int command_code) {
    return {motor.get_send_can_id(), pack_command_only(static_cast<uint8_t>(command_code & 0xFF))};
}

CANPacket CanPacketEncoder::create_refresh_command(const Motor& motor) {
    // Use 0xF1 to query MIT-mode state (q/dq/tau + status).
    return {motor.get_send_can_id(), pack_command_only(0xF1)};
}

CANPacket CanPacketEncoder::create_clear_fault_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_only(0xAF)};
}

CANPacket CanPacketEncoder::create_brake_command(const Motor& motor, uint8_t op) {
    return {motor.get_send_can_id(), pack_brake_control_data(op)};
}

CANPacket CanPacketEncoder::create_iq_control_command(const Motor& motor, double iq_current_a) {
    // Units: 0.001A, signed 32-bit
    int32_t milliamp = static_cast<int32_t>(std::llround(iq_current_a * 1000.0));
    return {motor.get_send_can_id(), pack_iq_control_data(milliamp)};
}

CANPacket CanPacketEncoder::create_speed_control_command(const Motor& motor, double speed_rpm) {
    // Units: 0.01rpm, signed 32-bit
    int32_t rpm_0p01 = static_cast<int32_t>(std::llround(speed_rpm * 100.0));
    return {motor.get_send_can_id(), pack_speed_control_data(rpm_0p01)};
}

CANPacket CanPacketEncoder::create_abs_pos_control_command_rad(const Motor& motor,
                                                              double position_rad) {
    // Units: Count, 1 rev = 16384 Count
    int32_t counts = static_cast<int32_t>(std::llround(position_rad * (16384.0 / (2.0 * M_PI))));
    return {motor.get_send_can_id(), pack_abs_pos_control_data(counts)};
}

CANPacket CanPacketEncoder::create_rel_pos_control_command_rad(const Motor& motor, double delta_rad) {
    int32_t delta_counts =
        static_cast<int32_t>(std::llround(delta_rad * (16384.0 / (2.0 * M_PI))));
    return {motor.get_send_can_id(), pack_rel_pos_control_data(delta_counts)};
}

CANPacket CanPacketEncoder::create_shortest_home_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_only(0xC4)};
}

CANPacket CanPacketEncoder::create_query_mit_limits_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_only(0xF0)};
}

CANPacket CanPacketEncoder::create_set_mit_limits_command(const Motor& motor, double pos_max_rad,
                                                          double vel_max_rad_s, double t_max_nm) {
    return {motor.get_send_can_id(), pack_mit_limits_set_data(pos_max_rad, vel_max_rad_s, t_max_nm)};
}

// Data interpretation methods (use recv_can_id for received data)
StateResult CanPacketDecoder::parse_motor_state_data(const Motor& motor,
                                                     const std::vector<uint8_t>& data) {
    // 0xF1 response is DLC=7, but other payloads might be 8 bytes.
    if (data.size() < 7) {
        std::cerr << "Warning: Skipping motor state data less than 7 bytes" << std::endl;
        return {0, 0, 0, 0, 0, false};
    }

    // MIT state payload is compatible with the common 16/12/12 layout.
    // In this protocol, 0xF1 responses are 0xF1 + 7 bytes where bytes[1..] match this layout.
    uint16_t q_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    uint16_t dq_uint =
        (static_cast<uint16_t>(data[3]) << 4) | (static_cast<uint16_t>(data[4]) >> 4);
    uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5];
    // data[6] is status in 0xF1 response. Temperatures are not provided here.
    int t_mos = 0;
    int t_rotor = 0;

    // Convert to physical values
    double recv_q = CanPacketDecoder::uint_to_double(q_uint, -motor.get_mit_pos_max_rad(),
                                                     motor.get_mit_pos_max_rad(), 16);
    double recv_dq = CanPacketDecoder::uint_to_double(dq_uint, -motor.get_mit_vel_max_rad_s(),
                                                      motor.get_mit_vel_max_rad_s(), 12);
    double recv_tau = CanPacketDecoder::uint_to_double(tau_uint, -motor.get_mit_t_max_nm(),
                                                       motor.get_mit_t_max_nm(), 12);

    return {recv_q, recv_dq, recv_tau, t_mos, t_rotor, true};
}

StateResult CanPacketDecoder::parse_a4_state_data(Motor& motor, const std::vector<uint8_t>& data) {
    // 0xA4 response: [0]=0xA4, [1]=temp(u8, °C), [2..3]=Iq(s16,0.001A), [4..5]=speed(s16,0.01rpm),
    // [6..7]=angle(u16, deg=val*(360/16384))
    if (data.size() < 8 || data[0] != 0xA4) {
        return {0, 0, 0, 0, 0, false};
    }

    const int t_mos = static_cast<int>(data[1]);
    const int16_t iq_mA = int16_from_le(data, 2);
    const int16_t rpm_0p01 = int16_from_le(data, 4);
    const uint16_t angle_u16 = uint16_from_le(data, 6);

    const double iq_a = static_cast<double>(iq_mA) / 1000.0;
    const double rpm = static_cast<double>(rpm_0p01) / 100.0;
    const double vel_rad_s = rpm * (2.0 * M_PI / 60.0);

    // single-turn angle in degrees -> radians
    const double angle_deg = static_cast<double>(angle_u16) * (360.0 / 16384.0);
    const double pos_rad = angle_deg * (M_PI / 180.0);

    motor.set_iq_current_a(iq_a);
    double tau_nm = 0.0;
    if (motor.has_torque_constant()) {
        tau_nm = motor.get_torque_constant_nm_per_a() * iq_a;
    }

    // Rotor temperature is not present in 0xA4
    return {pos_rad, vel_rad_s, tau_nm, t_mos, 0, true};
}

std::vector<ParamResult> CanPacketDecoder::parse_motor_param_data(Motor& motor,
                                                                  const std::vector<uint8_t>& data) {
    std::vector<ParamResult> out;
    if (data.empty()) return out;

    const uint8_t cmd = data[0];

    // 0xA1 / 0xC0 response: [0]=cmd, [1..4]=int32 (0.001A)
    if ((cmd == 0xA1 || cmd == 0xC0) && data.size() >= 5) {
        int32_t iq_mA = int32_from_le(data, 1);
        out.push_back({make_key(cmd, 1), static_cast<double>(iq_mA) / 1000.0, true});
        return out;
    }

    // 0xA2 / 0xC1 response: [0]=cmd, [1..4]=int32 (0.01rpm)
    if ((cmd == 0xA2 || cmd == 0xC1) && data.size() >= 5) {
        int32_t rpm_0p01 = int32_from_le(data, 1);
        out.push_back({make_key(cmd, 1), static_cast<double>(rpm_0p01) / 100.0, true});
        return out;
    }

    // 0xA3 / 0xC2 / 0xC3 / 0xC4 response: [0]=cmd, [1..2]=u16 angle, [3..6]=s32 total angle
    if ((cmd == 0xA3 || cmd == 0xC2 || cmd == 0xC3 || cmd == 0xC4) && data.size() >= 7) {
        uint16_t single_u16 = uint16_from_le(data, 1);
        int32_t multi_s32 = int32_from_le(data, 3);
        // Convert to degrees per doc: value*(360/16384)
        out.push_back({make_key(cmd, 1), static_cast<double>(single_u16) * (360.0 / 16384.0), true});
        out.push_back({make_key(cmd, 2), static_cast<double>(multi_s32) * (360.0 / 16384.0), true});
        return out;
    }

    // 0xA4 response: [0]=cmd, [1]=temp(C), [2..3]=i_q(s16,0.001A), [4..5]=speed(s16,0.01rpm), [6..7]=angle(u16)
    if (cmd == 0xA4 && data.size() >= 8) {
        uint8_t temp = data[1];
        int16_t iq_mA_s16 = int16_from_le(data, 2);
        int16_t rpm_0p01_s16 = int16_from_le(data, 4);
        uint16_t angle_u16 = uint16_from_le(data, 6);
        out.push_back({make_key(cmd, 1), static_cast<double>(temp), true});
        out.push_back({make_key(cmd, 2), static_cast<double>(iq_mA_s16) / 1000.0, true});
        out.push_back({make_key(cmd, 3), static_cast<double>(rpm_0p01_s16) / 100.0, true});
        out.push_back({make_key(cmd, 4), static_cast<double>(angle_u16) * (360.0 / 16384.0), true});
        return out;
    }

    // 0xAE response: [1..2]=busV(u16,0.01V), [3..4]=busI(u16,0.01A), [5]=temp, [6]=mode, [7]=fault
    if (cmd == 0xAE && data.size() >= 8) {
        uint16_t bus_v_0p01 = uint16_from_le(data, 1);
        uint16_t bus_i_0p01 = uint16_from_le(data, 3);
        uint8_t temp = data[5];
        uint8_t mode = data[6];
        uint8_t fault = data[7];
        out.push_back({make_key(cmd, 1), static_cast<double>(bus_v_0p01) / 100.0, true});
        out.push_back({make_key(cmd, 2), static_cast<double>(bus_i_0p01) / 100.0, true});
        out.push_back({make_key(cmd, 3), static_cast<double>(temp), true});
        out.push_back({make_key(cmd, 4), static_cast<double>(mode), true});
        out.push_back({make_key(cmd, 5), static_cast<double>(fault), true});
        return out;
    }

    // 0xAF response: [1]=fault
    if (cmd == 0xAF && data.size() >= 2) {
        out.push_back({make_key(cmd, 1), static_cast<double>(data[1]), true});
        return out;
    }

    // 0xB0 response: [1]=pole pairs(u8), [2..5]=Kt(float32), [6]=ratio(u8)
    if (cmd == 0xB0 && data.size() >= 7) {
        uint8_t npp = data[1];
        std::array<uint8_t, 4> kt_bytes = {data[2], data[3], data[4], data[5]};
        float kt = uint8s_to_float(kt_bytes);
        uint8_t ratio = data[6];
        out.push_back({make_key(cmd, 1), static_cast<double>(npp), true});
        out.push_back({make_key(cmd, 2), static_cast<double>(kt), true});
        out.push_back({make_key(cmd, 3), static_cast<double>(ratio), true});

        motor.set_pole_pairs(static_cast<int>(npp));
        motor.set_torque_constant_nm_per_a(static_cast<double>(kt));
        motor.set_gear_ratio(static_cast<double>(ratio));
        return out;
    }

    // 0xCE response: [1]=brake status
    if (cmd == 0xCE && data.size() >= 2) {
        out.push_back({make_key(cmd, 1), static_cast<double>(data[1]), true});
        return out;
    }

    // 0xF0 response: [1..2]=Pos_Max(0.1rad), [3..4]=Vel_Max(0.01rad/s), [5..6]=T_Max(0.01Nm)
    if (cmd == 0xF0 && data.size() >= 7) {
        uint16_t pos_0p1 = uint16_from_le(data, 1);
        uint16_t vel_0p01 = uint16_from_le(data, 3);
        uint16_t t_0p01 = uint16_from_le(data, 5);
        double pos = static_cast<double>(pos_0p1) / 10.0;
        double vel = static_cast<double>(vel_0p01) / 100.0;
        double t = static_cast<double>(t_0p01) / 100.0;
        out.push_back({make_key(cmd, 1), pos, true});
        out.push_back({make_key(cmd, 2), vel, true});
        out.push_back({make_key(cmd, 3), t, true});
        motor.set_mit_limits(pos, vel, t);
        return out;
    }

    // Unknown/unsupported cmd
    return out;
}

// Data packing utility methods
std::vector<uint8_t> CanPacketEncoder::pack_mit_control_data(const Motor& motor,
                                                             const MITParam& mit_param) {
    uint16_t kp_uint = double_to_uint(mit_param.kp, 0, 500, 12);
    uint16_t kd_uint = double_to_uint(mit_param.kd, 0, 5, 12);

    uint16_t q_uint = double_to_uint(mit_param.q, -motor.get_mit_pos_max_rad(),
                                     motor.get_mit_pos_max_rad(), 16);
    uint16_t dq_uint = double_to_uint(mit_param.dq, -motor.get_mit_vel_max_rad_s(),
                                      motor.get_mit_vel_max_rad_s(), 12);
    uint16_t tau_uint = double_to_uint(mit_param.tau, -motor.get_mit_t_max_nm(),
                                       motor.get_mit_t_max_nm(), 12);

    return {static_cast<uint8_t>((q_uint >> 8) & 0xFF),
            static_cast<uint8_t>(q_uint & 0xFF),
            static_cast<uint8_t>(dq_uint >> 4),
            static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
            static_cast<uint8_t>(kp_uint & 0xFF),
            static_cast<uint8_t>(kd_uint >> 4),
            static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
            static_cast<uint8_t>(tau_uint & 0xFF)};
}

std::vector<uint8_t> CanPacketEncoder::pack_abs_pos_control_data(int32_t counts) {
    // 0xC2: absolute position control, payload is int32 Count (little endian)
    std::vector<uint8_t> out;
    out.reserve(5);
    out.push_back(0xC2);
    append_int32_le(out, counts);
    return out;
}

std::vector<uint8_t> CanPacketEncoder::pack_iq_control_data(int32_t iq_milliamp) {
    // 0xC0: Q-axis current control, payload is int32 (0.001A units) little-endian
    std::vector<uint8_t> out;
    out.reserve(5);
    out.push_back(0xC0);
    append_int32_le(out, iq_milliamp);
    return out;
}

std::vector<uint8_t> CanPacketEncoder::pack_speed_control_data(int32_t rpm_0p01) {
    std::vector<uint8_t> out;
    out.reserve(5);
    out.push_back(0xC1);
    append_int32_le(out, rpm_0p01);
    return out;
}

std::vector<uint8_t> CanPacketEncoder::pack_rel_pos_control_data(int32_t delta_counts) {
    std::vector<uint8_t> out;
    out.reserve(5);
    out.push_back(0xC3);
    append_int32_le(out, delta_counts);
    return out;
}

std::vector<uint8_t> CanPacketEncoder::pack_brake_control_data(uint8_t op) {
    return {0xCE, op};
}

std::vector<uint8_t> CanPacketEncoder::pack_mit_limits_set_data(double pos_max_rad,
                                                                double vel_max_rad_s,
                                                                double t_max_nm) {
    // 0xF0 set: DLC=7 => [0]=0xF0, [1..2]=Pos_Max(0.1rad), [3..4]=Vel_Max(0.01rad/s), [5..6]=T_Max(0.01Nm)
    auto clamp_u16 = [](int64_t v) -> uint16_t {
        if (v < 0) v = 0;
        if (v > 0xFFFF) v = 0xFFFF;
        return static_cast<uint16_t>(v);
    };

    uint16_t pos_0p1 = clamp_u16(std::llround(pos_max_rad * 10.0));
    uint16_t vel_0p01 = clamp_u16(std::llround(vel_max_rad_s * 100.0));
    uint16_t t_0p01 = clamp_u16(std::llround(t_max_nm * 100.0));

    return {0xF0,
            static_cast<uint8_t>(pos_0p1 & 0xFF),
            static_cast<uint8_t>((pos_0p1 >> 8) & 0xFF),
            static_cast<uint8_t>(vel_0p01 & 0xFF),
            static_cast<uint8_t>((vel_0p01 >> 8) & 0xFF),
            static_cast<uint8_t>(t_0p01 & 0xFF),
            static_cast<uint8_t>((t_0p01 >> 8) & 0xFF)};
}

std::vector<uint8_t> CanPacketEncoder::pack_command_only(uint8_t cmd) { return {cmd}; }

// Utility function implementations
double CanPacketEncoder::limit_min_max(double x, double min, double max) {
    return std::max(min, std::min(x, max));
}

uint16_t CanPacketEncoder::double_to_uint(double x, double x_min, double x_max, int bits) {
    x = limit_min_max(x, x_min, x_max);
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

std::array<uint8_t, 4> CanPacketEncoder::float_to_uint8s(float value) {
    std::array<uint8_t, 4> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(float));
    return bytes;
}

void CanPacketEncoder::append_int32_le(std::vector<uint8_t>& out, int32_t value) {
    uint8_t bytes[4];
    std::memcpy(bytes, &value, sizeof(value));
    out.push_back(bytes[0]);
    out.push_back(bytes[1]);
    out.push_back(bytes[2]);
    out.push_back(bytes[3]);
}

double CanPacketDecoder::uint_to_double(uint16_t x, double min, double max, int bits) {
    double span = max - min;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    return data_norm * span + min;
}

float CanPacketDecoder::uint8s_to_float(const std::array<uint8_t, 4>& bytes) {
    float value;
    std::memcpy(&value, bytes.data(), sizeof(float));
    return value;
}

uint32_t CanPacketDecoder::uint8s_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3,
                                            uint8_t byte4) {
    uint32_t value;
    uint8_t bytes[4] = {byte1, byte2, byte3, byte4};
    std::memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}

bool CanPacketDecoder::is_in_ranges(int number) {
    return (7 <= number && number <= 10) || (13 <= number && number <= 16) ||
           (35 <= number && number <= 36);
}

int32_t CanPacketDecoder::int32_from_le(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 4 > data.size()) return 0;
    int32_t value;
    uint8_t bytes[4] = {data[offset + 0], data[offset + 1], data[offset + 2], data[offset + 3]};
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

int16_t CanPacketDecoder::int16_from_le(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 2 > data.size()) return 0;
    int16_t value;
    uint8_t bytes[2] = {data[offset + 0], data[offset + 1]};
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

uint16_t CanPacketDecoder::uint16_from_le(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 2 > data.size()) return 0;
    uint16_t value;
    uint8_t bytes[2] = {data[offset + 0], data[offset + 1]};
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}
}  // namespace openarm::oy_motor
