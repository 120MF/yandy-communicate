#ifndef YANDYCOMMUNICATENODE_HPP
#define YANDYCOMMUNICATENODE_HPP

#include <OF/lib/Node/Node.hpp>
#include <OF/lib/CommBridge/CommBridge.hpp>
#include <YandySlavePacket.hpp>
#include <zephyr/device.h>
#include <memory>

using namespace OF;

class YandyCommunicateNode : public Node<YandyCommunicateNode>
{
public:
    struct Meta
    {
        static constexpr size_t stack_size = 4096;
        static constexpr int priority = 5;
        static constexpr const char* name = "yandy_communicate";
    };

    struct Config
    {
        const device* uart_dev;
    };

    inline static Config config = {};

    bool init();
    void run();
    void cleanup();

private:
    std::unique_ptr<TxOnlyCommBridge<YandySlavePacket>> m_bridge;

    // Accumulated 6-axis values
    float m_accum_x = 0.3f;
    float m_accum_y = 0.0f;
    float m_accum_z = 0.55f;
    float m_accum_roll = 0.0f;
    float m_accum_pitch = 1.85f;
    float m_accum_yaw = 0.0f;

    // Previous key states for edge detection
    uint16_t m_prev_keys = 0;
    bool m_prev_ctrl = false;

    int m_stability_count = 0;
    bool key_event_processed = false;
    static constexpr int m_filter_threshold = 10;

    static constexpr float POS_RATE = 0.01f;
    static constexpr float ANG_RATE = 0.03f;
    static constexpr std::array<float, 3> MAX_POS = {0.4f, 0.4f, 0.8f};
    static constexpr float MAX_ANG = 3.14159f;

    void processRemoteControl(const struct VT03RemotePacket& remote);
    YandyControlCmd processKeyboardCmd(const struct VT03RemotePacket& remote);
};

#endif // YANDYCOMMUNICATENODE_HPP
