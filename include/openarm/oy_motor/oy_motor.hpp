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

#include <cstdint>
#include <cstring>
#include <map>

#include "oy_motor_constants.hpp"

namespace openarm::oy_motor {
class Motor {
    friend class OYCANDevice;  // Allow MotorDeviceCan to access protected
                               // members
    friend class OYControl;
    friend class CanPacketDecoder;

public:
    // Constructor
    Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id);

    // State getters (canonical internal units)
    // - position: rad
    // - velocity: rad/s
    // - torque: Nm (if Kt known, torque ~= Kt * Iq)
    // - temperatures: °C (if available)
    double get_position() const { return state_q_; }
    double get_velocity() const { return state_dq_; }
    double get_torque() const { return state_tau_; }
    int get_state_tmos() const { return state_tmos_; }
    int get_state_trotor() const { return state_trotor_; }

    // Latest electrical/diagnostic values (when available)
    double get_iq_current_a() const { return iq_current_a_; }
    bool has_torque_constant() const { return torque_constant_nm_per_a_valid_; }
    double get_torque_constant_nm_per_a() const { return torque_constant_nm_per_a_; }
    bool has_gear_ratio() const { return gear_ratio_valid_; }
    double get_gear_ratio() const { return gear_ratio_; }
    bool has_pole_pairs() const { return pole_pairs_valid_; }
    int get_pole_pairs() const { return pole_pairs_; }

    // Motor property getters
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }

    // MIT (运控模式) limits used for encoding/decoding (configurable via 0xF0)
    double get_mit_pos_max_rad() const { return mit_pos_max_rad_; }
    double get_mit_vel_max_rad_s() const { return mit_vel_max_rad_s_; }
    double get_mit_t_max_nm() const { return mit_t_max_nm_; }
    void set_mit_limits(double pos_max_rad, double vel_max_rad_s, double t_max_nm) {
        mit_pos_max_rad_ = pos_max_rad;
        mit_vel_max_rad_s_ = vel_max_rad_s;
        mit_t_max_nm_ = t_max_nm;
    }

    // Enable status getters
    bool is_enabled() const { return enabled_; }

    // Parameter methods
    double get_param(int RID) const;

    // Static methods for motor properties
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // State update methods
    void update_state(double q, double dq, double tau, int tmos, int trotor);
    void set_state_tmos(int tmos);
    void set_state_trotor(int trotor);
    void set_enabled(bool enabled);
    void set_temp_param(int key, double val);

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // Enable status
    bool enabled_;

    // Current state
    double state_q_, state_dq_, state_tau_; //位置 rad 速度 rad/s 扭矩 N·m
    int state_tmos_, state_trotor_; // MOS温度 摄氏度 转子温度 摄氏度

    // Parameter storage
    std::map<int, double> temp_param_dict_;

    // Protocol defaults (doc Rev.3.07b0): Pos_Max=95.5rad, Vel_Max=45.00rad/s, T_Max=18.00Nm
    // These can be overridden by parsing 0xF0 responses.
    double mit_pos_max_rad_{95.5};
    double mit_vel_max_rad_s_{45.0};
    double mit_t_max_nm_{18.0};

    // Protocol-derived parameters
    double iq_current_a_{0.0};
    double torque_constant_nm_per_a_{0.0};
    bool torque_constant_nm_per_a_valid_{false};
    double gear_ratio_{0.0};
    bool gear_ratio_valid_{false};
    int pole_pairs_{0};
    bool pole_pairs_valid_{false};

    void set_iq_current_a(double a) { iq_current_a_ = a; }
    void set_torque_constant_nm_per_a(double kt) {
        torque_constant_nm_per_a_ = kt;
        torque_constant_nm_per_a_valid_ = true;
    }
    void set_gear_ratio(double ratio) {
        gear_ratio_ = ratio;
        gear_ratio_valid_ = true;
    }
    void set_pole_pairs(int npp) {
        pole_pairs_ = npp;
        pole_pairs_valid_ = true;
    }

protected:
    uint16_t boot_version_{0};
    uint16_t app_version_{0};
    uint16_t hw_version_{0};
    uint8_t can_protocol_version_{0};
    void update_versions(uint16_t boot, uint16_t app, uint16_t hw, uint8_t can_proto) {
        boot_version_ = boot;
        app_version_ = app;
        hw_version_ = hw;
        can_protocol_version_ = can_proto;
    }


};
}  // namespace openarm::oy_motor
