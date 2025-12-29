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
