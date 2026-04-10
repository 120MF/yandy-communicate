#include <YandyCommunicateNode.hpp>
#include <YandyCommunicateData.hpp>
#include <YandyControllerPacket.hpp>
#include <YandyGimbalData.hpp>

#include <OF/lib/VtHub/VtHub.hpp>
#include <RPL/Packets/VT03RemotePacket.hpp>
#include <RPL/Packets/RoboMaster/CustomControllerData.hpp>

#include <zephyr/logging/log.h>
#include <cmath>
#include <cstring>

#include "OF/lib/ImuHub/ImuHub.hpp"

LOG_MODULE_REGISTER(YandyCommunicateNode, CONFIG_YANDY_COMMUNICATE_LOG_LEVEL);

using namespace OF;

// External Topic reference for gimbal data
extern Topic<YandyGimbalData>& topic_yandy_gimbal;

bool YandyCommunicateNode::init()
{
    if (config.uart_dev == nullptr || !device_is_ready(config.uart_dev))
    {
        LOG_ERR("UART device not ready");
        return false;
    }

    m_bridge = TxOnlyCommBridge<YandySlavePacket>::create(config.uart_dev);
    if (!m_bridge)
    {
        LOG_ERR("Failed to create CommBridge");
        return false;
    }

    LOG_INF("YandyCommunicateNode initialized");
    return true;
}

void YandyCommunicateNode::run()
{
    LOG_INF("YandyCommunicateNode running");

    while (true)
    {
        k_sleep(K_MSEC(4)); // ~250Hz

        auto remote_result = VtHub::get<VT03RemotePacket>();
        if (!remote_result)
        {
            continue;
        }

        auto remote = remote_result.value();
        const float wheel = -vt_stick_percent(remote.wheel());

        bool use_custom_controller = false;
        // Get gimbal data
        auto gimbal_data = topic_yandy_gimbal.read();
        m_packet.gimbal_z = gimbal_data.gimbal_z;
        m_packet.gimbal_yaw = gimbal_data.gimbal_yaw;
        m_packet.gimbal_pitch = gimbal_data.gimbal_pitch;

        // Get IMU data and apply low-pass filter to accel and gyro
        const auto imu_data = ImuHub::getData();

        // Apply low-pass filter to accelerometer data
        m_filtered_imu.accel_x = lowPassFilter(imu_data.accel.x, m_filtered_imu.accel_x, ACCEL_ALPHA);
        m_filtered_imu.accel_y = lowPassFilter(imu_data.accel.y, m_filtered_imu.accel_y, ACCEL_ALPHA);
        m_filtered_imu.accel_z = lowPassFilter(imu_data.accel.z, m_filtered_imu.accel_z, ACCEL_ALPHA);

        // Apply low-pass filter to gyroscope data
        m_filtered_imu.gyro_x = lowPassFilter(imu_data.gyro.x, m_filtered_imu.gyro_x, GYRO_ALPHA);
        m_filtered_imu.gyro_y = lowPassFilter(imu_data.gyro.y, m_filtered_imu.gyro_y, GYRO_ALPHA);
        m_filtered_imu.gyro_z = lowPassFilter(imu_data.gyro.z, m_filtered_imu.gyro_z, GYRO_ALPHA);

        // Use filtered data for packet - IMU fields
        m_packet.base_ax = m_filtered_imu.accel_x;
        m_packet.base_ay = m_filtered_imu.accel_y;
        m_packet.base_az = m_filtered_imu.accel_z;
        m_packet.base_gx = m_filtered_imu.gyro_x;
        m_packet.base_gy = m_filtered_imu.gyro_y;
        m_packet.base_gz = m_filtered_imu.gyro_z;
        m_packet.base_qw = imu_data.quat.w;
        m_packet.base_qx = imu_data.quat.x;
        m_packet.base_qy = imu_data.quat.y;
        m_packet.base_qz = imu_data.quat.z;

        // Process keyboard commands (always active)
        m_packet.cmd = processKeyboardCmd(remote);

        bool special_pos_triggered = false;


        if (remote.switch_state() == 2)
        {
            if (remote.key_shift() && remote.key_c())
            {
                if (m_stability_count < m_filter_threshold)
                {
                    ++m_stability_count;
                }
                else
                {
                    if (!key_event_processed)
                    {
                        use_custom_controller = !use_custom_controller;
                        key_event_processed = true;
                    }
                }
            }
            else
            {
                m_stability_count = 0;
                key_event_processed = false;
            }

            if (!use_custom_controller)
            {
                // Remote control mode: accumulate 6-axis values
                processRemoteControl(remote);
                if (remote.custom_right())
                {
                    m_accum_x = 0.25;
                    m_accum_y = -0.05;
                    m_accum_z = 0.7;
                    m_accum_qx = 0.53;














                    m_accum_qy = -0.21;
                    m_accum_qz = 0.8;
                    m_accum_qw = -0.21;
                }

                m_packet.x = m_accum_x;
                m_packet.y = m_accum_y;
                m_packet.z = m_accum_z;
                m_packet.qw = m_accum_qw;
                m_packet.qx = m_accum_qx;
                m_packet.qy = m_accum_qy;
                m_packet.qz = m_accum_qz;

                if (remote.trigger())
                {
                    m_packet.cmd = YandyControlCmd::CMD_SWITCH_ENABLE;
                }
            }
            else
            {
                // Custom controller mode
                auto controller_result = VtHub::get<CustomControllerData>();
                if (controller_result)
                {
                    // Directly memcpy 24 bytes (6 floats) from custom controller data
                    std::memcpy(&m_packet.x, controller_result.value().data, sizeof(YandyControllerPacket));
                }
            }
            LOG_INF("%.2f, %.2f, %.2f; %.2f, %.2f, %.2f, %.2f", m_accum_x, m_accum_y, m_accum_z, m_accum_qx, m_accum_qy,
                    m_accum_qz, m_accum_qw);
        }
        else if (remote.switch_state() == 1)
        {
            if (remote.pause_btn())
            {
                m_packet.cmd = YandyControlCmd::CMD_TOGGLE_HELD;
            }
            else if (remote.custom_left())
            {
                m_packet.cmd = YandyControlCmd::CMD_DEC_STORE;
            }
            else if (remote.custom_right())
            {
                m_packet.cmd = YandyControlCmd::CMD_INC_STORE;
            }
            else if (remote.trigger())
            {
                m_packet.cmd = YandyControlCmd::CMD_SWITCH_GRIP;
            }
        }

        else if (remote.switch_state() == 0)
        {
            // 1. Handle Trigger Reset and Special Positions (Highest Priority in State 0)
            if (remote.trigger())
            {
                m_accum_x = m_init_x;
                m_accum_y = m_init_y;
                m_accum_z = m_init_z;
                m_accum_qw = m_init_qw;
                m_accum_qx = m_init_qx;
                m_accum_qy = m_init_qy;
                m_accum_qz = m_init_qz;
                special_pos_triggered = true;
            }
            else if (remote.pause_btn())
            {
                m_accum_x = 0.25f;
                m_accum_y = -0.11f;
                m_accum_z = 0.75f;
                m_accum_qw = 0.0f;
                m_accum_qx = -0.07f;
                m_accum_qy = 0.0f;
                m_accum_qz = 1.0f;
                special_pos_triggered = true;
            }
            else if (remote.custom_left())
            {
                m_accum_x = 0.32f;
                m_accum_y = 0.11f;
                m_accum_z = 0.48f;
                m_accum_qw = 0.66f;
                m_accum_qx = 0.27f;
                m_accum_qy = 0.61f;
                m_accum_qz = 0.33f;
                special_pos_triggered = true;
            }
            else if (remote.custom_right())
            {
                m_accum_x = 0.37f;
                m_accum_y = -0.15f;
                m_accum_z = 0.50f;
                m_accum_qw = 0.53f;
                m_accum_qx = -0.37f;
                m_accum_qy = 0.73f;
                m_accum_qz = -0.23f;
                special_pos_triggered = true;
            }

            if (special_pos_triggered)
            {
                m_packet.x = m_accum_x;
                m_packet.y = m_accum_y;
                m_packet.z = m_accum_z;
                m_packet.qw = m_accum_qw;
                m_packet.qx = m_accum_qx;
                m_packet.qy = m_accum_qy;
                m_packet.qz = m_accum_qz;

                m_playback_cursor = -2; // Special value: "Reset/Manual state", don't auto-grab history
            }
            // 2. Handle History Playback via Wheel
            else if (m_recorded_count > 0)
            {
                // Wheel Flick Left (wheel--): Step back 0.5s
                if (wheel < -0.5f && m_prev_wheel >= -0.5f)
                {
                    if (m_playback_cursor < 0)
                    {
                        // First time or after trigger/special: start from latest
                        m_playback_cursor = (m_history_head - 1 + HISTORY_SIZE) % HISTORY_SIZE;
                    }
                    else
                    {
                        // Step back 125 frames (0.5s)
                        m_playback_cursor = (m_playback_cursor - 125 + HISTORY_SIZE) % HISTORY_SIZE;
                    }
                }
                // Wheel Flick Right (wheel++): Jump to latest
                else if (wheel > 0.5f && m_prev_wheel <= 0.5f)
                {
                    m_playback_cursor = (m_history_head - 1 + HISTORY_SIZE) % HISTORY_SIZE;
                }

                // If cursor is active, apply the frame
                if (m_playback_cursor >= 0)
                {
                    const auto& frame = m_history_buffer[m_playback_cursor];
                    m_accum_x = frame.x;
                    m_accum_y = frame.y;
                    m_accum_z = frame.z;
                    m_accum_qw = frame.qw;
                    m_accum_qx = frame.qx;
                    m_accum_qy = frame.qy;
                    m_accum_qz = frame.qz;

                    m_packet.x = m_accum_x;
                    m_packet.y = m_accum_y;
                    m_packet.z = m_accum_z;
                    m_packet.qw = m_accum_qw;
                    m_packet.qx = m_accum_qx;
                    m_packet.qy = m_accum_qy;
                    m_packet.qz = m_accum_qz;
                }
            }
        }

        // Record history if not in playback mode (switch_state != 0)
        if (remote.switch_state() != 0)
        {
            m_history_buffer[m_history_head] = {
                m_accum_x, m_accum_y, m_accum_z,
                m_accum_qw, m_accum_qx, m_accum_qy, m_accum_qz
            };
            m_history_head = (m_history_head + 1) % HISTORY_SIZE;
            if (m_recorded_count < HISTORY_SIZE)
            {
                m_recorded_count++;
            }
            m_playback_cursor = -1;
        }

        m_prev_wheel = wheel;

        // Send the packet

        m_bridge->send(m_packet);
    }
}

void YandyCommunicateNode::cleanup()
{
}

void YandyCommunicateNode::eulerToQuaternion(float roll, float pitch, float yaw, float& qw, float& qx, float& qy,
                                             float& qz)
{
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

void YandyCommunicateNode::multiplyQuaternion(float w1, float x1, float y1, float z1,
                                              float w2, float x2, float y2, float z2,
                                              float& qw, float& qx, float& qy, float& qz)
{
    qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

void YandyCommunicateNode::rotateVectorByQuaternion(float vx, float vy, float vz,
                                                    float qw, float qx, float qy, float qz,
                                                    float& rx, float& ry, float& rz)
{
    // t = 2 * cross(q.xyz, v)
    float tx = 2.0f * (qy * vz - qz * vy);
    float ty = 2.0f * (qz * vx - qx * vz);
    float tz = 2.0f * (qx * vy - qy * vx);

    // r = v + q.w * t + cross(q.xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty);
    ry = vy + qw * ty + (qz * tx - qx * tz);
    rz = vz + qw * tz + (qx * ty - qy * tx);
}

void YandyCommunicateNode::processRemoteControl(const VT03RemotePacket& remote)
{
    const float lx = -vt_stick_percent(remote.left_stick_x());
    const float ly = vt_stick_percent(remote.left_stick_y());
    const float wheel = -vt_stick_percent(remote.wheel());
    const float rx = vt_stick_percent(remote.right_stick_x());
    const float ry = vt_stick_percent(remote.right_stick_y());

    // Local angle increments
    float d_roll = rx * ANG_RATE;
    float d_pitch = ry * ANG_RATE;
    float d_yaw = 0.0f;
    float dx_local_input = 0.0f;

    if (remote.pause_btn())
    {
        d_yaw = wheel * ANG_RATE;
    }
    else
    {
        dx_local_input = wheel;
    }

    // Convert local angle increments to delta quaternion
    float dq_w, dq_x, dq_y, dq_z;
    eulerToQuaternion(d_roll, d_pitch, d_yaw, dq_w, dq_x, dq_y, dq_z);

    // Accumulate orientation: new_Q = old_Q * delta_Q (local rotation)
    float new_qw, new_qx, new_qy, new_qz;
    multiplyQuaternion(m_accum_qw, m_accum_qx, m_accum_qy, m_accum_qz,
                       dq_w, dq_x, dq_y, dq_z,
                       new_qw, new_qx, new_qy, new_qz);

    // Normalize quaternion to prevent drift
    float mag = std::sqrt(new_qw * new_qw + new_qx * new_qx + new_qy * new_qy + new_qz * new_qz);
    if (mag > 0.0001f)
    {
        m_accum_qw = new_qw / mag;
        m_accum_qx = new_qx / mag;
        m_accum_qy = new_qy / mag;
        m_accum_qz = new_qz / mag;
    }

    // Accumulate position
    if (remote.custom_left())
    {
        // Global coordinate accumulation mode
        m_accum_x = std::clamp(m_accum_x + ly * POS_RATE, -MAX_POS[0], MAX_POS[0]);
        m_accum_y = std::clamp(m_accum_y + lx * POS_RATE, -MAX_POS[1], MAX_POS[1]);
        if (!remote.pause_btn())
        {
            m_accum_z = std::clamp(m_accum_z + wheel * POS_RATE, -MAX_POS[2], MAX_POS[2]);
        }
    }
    else
    {
        // Local coordinate accumulation mode (Rotated)
        const float dx_local = dx_local_input * POS_RATE;
        const float dy_local = lx * POS_RATE;
        const float dz_local = ly * POS_RATE;

        float dx_global, dy_global, dz_global;
        rotateVectorByQuaternion(dx_local, dy_local, dz_local,
                                 m_accum_qw, m_accum_qx, m_accum_qy, m_accum_qz,
                                 dx_global, dy_global, dz_global);

        m_accum_x = std::clamp(m_accum_x + dx_global, -MAX_POS[0], MAX_POS[0]);
        m_accum_y = std::clamp(m_accum_y + dy_global, -MAX_POS[1], MAX_POS[1]);
        m_accum_z = std::clamp(m_accum_z + dz_global, -MAX_POS[2], MAX_POS[2]);
    }
}

YandyControlCmd YandyCommunicateNode::processKeyboardCmd(const VT03RemotePacket& remote)
{
    // Build current key state bitmask
    uint16_t keys = 0;
    if (remote.key_f()) keys |= (1 << 0);
    if (remote.key_r()) keys |= (1 << 1);
    if (remote.key_g()) keys |= (1 << 2);
    if (remote.key_c()) keys |= (1 << 3);
    if (remote.key_z()) keys |= (1 << 4);
    if (remote.key_b()) keys |= (1 << 5);
    if (remote.key_e()) keys |= (1 << 6);
    if (remote.key_q()) keys |= (1 << 7);

    bool ctrl = remote.key_ctrl();

    // Detect rising edges (newly pressed keys)
    uint16_t pressed = keys & ~m_prev_keys;

    // Update previous state
    m_prev_keys = keys;
    m_prev_ctrl = ctrl;

    // Priority: Check Ctrl combinations first
    if (ctrl)
    {
        // Ctrl + F: Toggle held state
        if (pressed & (1 << 0))
        {
            return YandyControlCmd::CMD_TOGGLE_HELD;
        }
        // Ctrl + E: Increase store
        if (pressed & (1 << 6))
        {
            return YandyControlCmd::CMD_INC_STORE;
        }
        // Ctrl + Q: Decrease store
        if (pressed & (1 << 7))
        {
            return YandyControlCmd::CMD_DEC_STORE;
        }
    }
    else
    {
        // Single key commands (only on rising edge)
        // F: Fetch
        if (pressed & (1 << 0))
        {
            return YandyControlCmd::CMD_SWITCH_FETCH;
        }
        // R: Store
        if (pressed & (1 << 1))
        {
            return YandyControlCmd::CMD_SWITCH_STORE;
        }
        // G: Enable
        if (pressed & (1 << 2))
        {
            return YandyControlCmd::CMD_SWITCH_ENABLE;
        }
        // C: Grip
        if (pressed & (1 << 3))
        {
            return YandyControlCmd::CMD_SWITCH_GRIP;
        }
        // Z: Reset
        if (pressed & (1 << 4))
        {
            return YandyControlCmd::CMD_RESET;
        }
        // B: Error/Emergency stop
        if (pressed & (1 << 5))
        {
            return YandyControlCmd::CMD_ERROR;
        }
    }

    return YandyControlCmd::CMD_NONE;
}

// Register the Node with OneFramework
ONE_NODE_REGISTER(YandyCommunicateNode);
