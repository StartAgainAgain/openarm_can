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

#include <array>
#include <cstddef>
#include <cstdint>

namespace openarm::oy_motor {
enum class MotorType : uint8_t {
    GIM8115_9p = 0,
    GIM4310_40 = 1,
    GIM4315_8 = 2,
    COUNT = 3
};

enum class ControlMode : uint8_t { MIT = 1, POS_VEL = 2, VEL = 3, TORQUE_POS = 4 };

enum class RID : uint8_t {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
    COUNT = 82
};

enum class CMDCODE : uint8_t {
    // System
    // REBOOT = 0x00,           // reboot slave (no response)
    //status read
    READ_VERSION = 0xA0,     // read Boot/App/HW/CAN-proto version
    READ_IQ = 0xA1,          // read Q-axis current
    READ_SPEED = 0xA2,       // read rotating speed
    READ_ANGLE = 0xA3,       // read single-turn + multi-turn angle
    READ_A4_STATE = 0xA4,    // read temperature + iq + speed + single-turn angle
    READ_AE_STATUS = 0xAE,   // read busV/busI/temp/mode/fault (may be unsolicited)
    CLEAR_FAULT = 0xAF,      // clear fault

    // Parameters Read/Set
    READ_MOTOR_PARAMS = 0xB0,        // read pole pairs / torque constant / gear ratio
    SET_ZERO = 0xB1,                 // set current position as origin
    SET_POS_MODE_MAX_SPEED = 0xB2,   // set position-mode max speed (volatile)
    SET_MAX_IQ = 0xB3,               // set max Q-axis current (volatile)
    SET_IQ_SLOPE = 0xB4,             // set iq slope (volatile)
    SET_SPEED_ACCEL = 0xB5,          // set speed acceleration (volatile)
    POS_KP = 0xB6,                   // read/set position loop Kp (volatile)
    POS_KI = 0xB7,                   // read/set position loop Ki (volatile)
    SPEED_KP = 0xB8,                 // read/set speed loop Kp (volatile)
    SPEED_KI = 0xB9,                 // read/set speed loop Ki (volatile)

    // Control
    IQ_CONTROL = 0xC0,        // iq control
    SPEED_CONTROL = 0xC1,     // speed control
    ABS_POS_CONTROL = 0xC2,   // absolute position control
    REL_POS_CONTROL = 0xC3,   // relative position control
    SHORTEST_HOME = 0xC4,     // shortest distance to home
    BRAKE_CONTROL = 0xCE,     // brake output control
    DISABLE_OUTPUT = 0xCF,    // disable motor output (free state)

    // MIT-mode
    MIT_LIMITS = 0xF0,        // read/set Pos_Max/Vel_Max/T_Max
    MIT_STATE = 0xF1,         // read MIT state
    // NOTE: MIT control command has NO command code byte; it uses StdID Bit[10]=1 (OR with 0x400).
};

// Limit parameters structure for different motor types
struct LimitParam {
    double pMax;  // Position limit (rad)
    double vMax;  // Velocity limit (rad/s)
    double tMax;  // Torque limit (Nm)
};
// Limit parameters for each motor type [pMax, vMax, tMax]
inline constexpr std::array<LimitParam, static_cast<std::size_t>(MotorType::COUNT)>
    MOTOR_LIMIT_PARAMS = {{
        {12.5, 30, 10},   // GIM4310_40
        {12.5, 50, 10},   // GIM4315_8
        {12.5, 8, 28},    // GIM8115_9p
    }};
}  // namespace openarm::oy_motor
