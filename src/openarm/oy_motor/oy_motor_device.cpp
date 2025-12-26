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
#include <iostream>
#include <openarm/oy_motor/oy_motor.hpp>
#include <openarm/oy_motor/oy_motor_constants.hpp>
#include <openarm/oy_motor/oy_motor_control.hpp>
#include <openarm/oy_motor/oy_motor_device.hpp>

namespace openarm::oy_motor {

OYCANDevice::OYCANDevice(Motor& motor, canid_t recv_can_mask, bool use_fd)
    : canbus::CANDevice(motor.get_send_can_id(), motor.get_recv_can_id(), recv_can_mask, use_fd),
      motor_(motor),
      callback_mode_(CallbackMode::STATE),
      use_fd_(use_fd) {}

std::vector<uint8_t> OYCANDevice::get_data_from_frame(const can_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
}

std::vector<uint8_t> OYCANDevice::get_data_from_frame(const canfd_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.len);
}
void OYCANDevice::callback(const can_frame& frame) {
    if (use_fd_) {
        std::cerr << "WARNING: WRONG CALLBACK FUNCTION" << std::endl;
        return;
    }

    std::vector<uint8_t> data = get_data_from_frame(frame);

    // Protocol: fault/status (0xAE) may be reported periodically without request.
    // Treat it as PARAM-like data regardless of current callback mode.
    if (!data.empty() && data[0] == 0xAE) {
        auto results = CanPacketDecoder::parse_motor_param_data(motor_, data);
        for (const auto& r : results) {
            if (r.valid) motor_.set_temp_param(r.rid, r.value);
        }
        return;
    }

    // Unsolicited fault/status reporting (0xAE) can arrive at any time.
    if (!data.empty() && data[0] == 0xAE) {
        auto results = CanPacketDecoder::parse_motor_param_data(motor_, data);
        for (const auto& r : results) {
            if (r.valid) motor_.set_temp_param(r.rid, r.value);
        }
        return;
    }

    switch (callback_mode_) {
        case STATE:
            if (frame.can_id != motor_.get_recv_can_id() || data.empty()) break;

            // 0xF1 response: MIT state (DLC=7)
            if (data[0] == 0xF1 && frame.can_dlc >= 7) {
                StateResult result = CanPacketDecoder::parse_motor_state_data(motor_, data);
                if (result.valid) {
                    motor_.update_state(result.position, result.velocity, result.torque,
                                        result.t_mos, result.t_rotor);
                    // Store status bits from 0xF1 in temp params
                    motor_.set_temp_param((0xF1 << 8) | 1, static_cast<double>(data[6]));
                }
                break;
            }

            // 0xA4 response: temperature/current/speed/angle (DLC=8)
            if (data[0] == 0xA4 && frame.can_dlc >= 8) {
                StateResult result = CanPacketDecoder::parse_a4_state_data(motor_, data);
                if (result.valid) {
                    motor_.update_state(result.position, result.velocity, result.torque,
                                        result.t_mos, result.t_rotor);
                }
                break;
            }
            break;
        case PARAM: {
            auto results = CanPacketDecoder::parse_motor_param_data(motor_, data);
            for (const auto& r : results) {
                if (r.valid) motor_.set_temp_param(r.rid, r.value);
            }
            break;
        }
        case IGNORE:
            return;
        default:
            break;
    }
}

void OYCANDevice::callback(const canfd_frame& frame) {
    if (not use_fd_) {
        std::cerr << "WARNING: CANFD MODE NOT ENABLED" << std::endl;
        return;
    }

    if (frame.can_id != motor_.get_recv_can_id()) {
        std::cerr << "WARNING: CANFD FRAME ID DOES NOT MATCH MOTOR ID" << std::endl;
        return;
    }

    std::vector<uint8_t> data = get_data_from_frame(frame);
    if (!data.empty() && data[0] == 0xAE) {
        auto results = CanPacketDecoder::parse_motor_param_data(motor_, data);
        for (const auto& r : results) {
            if (r.valid) motor_.set_temp_param(r.rid, r.value);
        }
        return;
    }
    if (callback_mode_ == STATE) {
        if (!data.empty() && data[0] == 0xF1 && frame.len >= 7) {
            StateResult result = CanPacketDecoder::parse_motor_state_data(motor_, data);
            if (result.valid) {
                motor_.update_state(result.position, result.velocity, result.torque, result.t_mos,
                                    result.t_rotor);
                motor_.set_temp_param((0xF1 << 8) | 1, static_cast<double>(data[6]));
            }
        } else if (!data.empty() && data[0] == 0xA4 && frame.len >= 8) {
            StateResult result = CanPacketDecoder::parse_a4_state_data(motor_, data);
            if (result.valid) {
                motor_.update_state(result.position, result.velocity, result.torque, result.t_mos,
                                    result.t_rotor);
            }
        }
    } else if (callback_mode_ == PARAM) {
        auto results = CanPacketDecoder::parse_motor_param_data(motor_, data);
        for (const auto& r : results) {
            if (r.valid) motor_.set_temp_param(r.rid, r.value);
        }
    } else if (callback_mode_ == IGNORE) {
        return;
    }
}

can_frame OYCANDevice::create_can_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = send_can_id;
    frame.can_dlc = data.size();
    std::copy(data.begin(), data.end(), frame.data);
    return frame;
}

canfd_frame OYCANDevice::create_canfd_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    canfd_frame frame;
    frame.can_id = send_can_id;
    frame.len = data.size();
    frame.flags = CANFD_BRS;
    std::copy(data.begin(), data.end(), frame.data);
    return frame;
}

}  // namespace openarm::oy_motor
