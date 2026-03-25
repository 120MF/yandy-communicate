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
        bool use_custom_controller = false;
        if (remote.switch_state == 2)
        {
            YandySlavePacket packet{};

            // Get gimbal data
            auto gimbal_data = topic_yandy_gimbal.read();
            packet.gimbal_z = gimbal_data.gimbal_z;
            packet.gimbal_yaw = gimbal_data.gimbal_yaw;
            packet.gimbal_pitch = gimbal_data.gimbal_pitch;

            if (remote.key_shift && remote.key_c)
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

                packet.x = m_accum_x;
                packet.y = m_accum_y;
                packet.z = m_accum_z;
                packet.roll = m_accum_roll;
                packet.pitch = m_accum_pitch;
                packet.yaw = m_accum_yaw;
            }
            else
            {
                // Custom controller mode
                auto controller_result = VtHub::get<CustomControllerData>();
                if (controller_result)
                {
                    // Directly memcpy 24 bytes (6 floats) from custom controller data
                    std::memcpy(&packet.x, controller_result.value().data, sizeof(YandyControllerPacket));
                }
            }

            // Process keyboard commands (always active)
            packet.cmd = processKeyboardCmd(remote);
            // Send the packet
            m_bridge->send(packet);
        }
    }
}

void YandyCommunicateNode::cleanup()
{
}

void YandyCommunicateNode::processRemoteControl(const VT03RemotePacket& remote)
{
    const float lx = vt_stick_percent(remote.left_stick_x);
    const float ly = vt_stick_percent(remote.left_stick_y);
    const float wheel = vt_stick_percent(remote.wheel);
    const float rx = vt_stick_percent(remote.right_stick_x);
    const float ry = vt_stick_percent(remote.right_stick_y);

    m_accum_yaw = std::clamp(m_accum_yaw + rx * ANG_RATE, -MAX_ANG, MAX_ANG);
    m_accum_pitch = std::clamp(m_accum_pitch + ry * ANG_RATE, -MAX_ANG, MAX_ANG);

    float dz_local_input = 0.0f;
    if (remote.pause_btn)
    {
        m_accum_roll = std::clamp(m_accum_roll + wheel * ANG_RATE, -MAX_ANG, MAX_ANG);
    }
    else
    {
        dz_local_input = wheel; // 滚轮控制局部Z轴
    }

    const float dx_local = lx * POS_RATE;
    const float dy_local = ly * POS_RATE;
    const float dz_local = dz_local_input * POS_RATE;

    const float cy = std::cos(m_accum_yaw);
    const float sy = std::sin(m_accum_yaw);
    const float cp = std::cos(m_accum_pitch);
    const float sp = std::sin(m_accum_pitch);
    const float cr = std::cos(m_accum_roll);
    const float sr = std::sin(m_accum_roll);

    const float dx_global = (cy * cp) * dx_local
        + (cy * sp * sr - sy * cr) * dy_local
        + (cy * sp * cr + sy * sr) * dz_local;

    const float dy_global = (sy * cp) * dx_local
        + (sy * sp * sr + cy * cr) * dy_local + (sy * sp * cr - cy * sr) * dz_local;

    const float dz_global = (-sp) * dx_local
        + (cp * sr) * dy_local
        + (cp * cr) * dz_local;

    m_accum_x = std::clamp(m_accum_x + dx_global, -MAX_POS[0], MAX_POS[0]);
    m_accum_y = std::clamp(m_accum_y + dy_global, -MAX_POS[1], MAX_POS[1]);
    m_accum_z = std::clamp(m_accum_z + dz_global, -MAX_POS[2], MAX_POS[2]);
}

YandyControlCmd YandyCommunicateNode::processKeyboardCmd(const VT03RemotePacket& remote)
{
    // Build current key state bitmask
    uint16_t keys = 0;
    if (remote.key_f) keys |= (1 << 0);
    if (remote.key_r) keys |= (1 << 1);
    if (remote.key_g) keys |= (1 << 2);
    if (remote.key_c) keys |= (1 << 3);
    if (remote.key_z) keys |= (1 << 4);
    if (remote.key_b) keys |= (1 << 5);
    if (remote.key_e) keys |= (1 << 6);
    if (remote.key_q) keys |= (1 << 7);

    bool ctrl = remote.key_ctrl;

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
