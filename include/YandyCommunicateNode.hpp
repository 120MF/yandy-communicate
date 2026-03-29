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

    static constexpr float m_init_x = 0.3f;
    static constexpr float m_init_y = 0.0f;
    static constexpr float m_init_z = 0.55f;
    static constexpr float m_init_roll = 0.0f;
    static constexpr float m_init_pitch = 1.85f;
    static constexpr float m_init_yaw = 0.0f;

    // Helper functions for quaternion math
    static void eulerToQuaternion(float roll, float pitch, float yaw, float& qw, float& qx, float& qy, float& qz);
    static void multiplyQuaternion(float w1, float x1, float y1, float z1,
                                   float w2, float x2, float y2, float z2,
                                   float& qw, float& qx, float& qy, float& qz);
    static void rotateVectorByQuaternion(float vx, float vy, float vz,
                                         float qw, float qx, float qy, float qz,
                                         float& rx, float& ry, float& rz);

    // Initial quaternion computed from euler angles
    static constexpr float m_init_qw = 0.601429f, m_init_qx = 0.0f, m_init_qy = 0.798926f, m_init_qz = 0.0f;
    // from pitch=1.85

    // Accumulated values
    float m_accum_x = m_init_x;
    float m_accum_y = m_init_y;
    float m_accum_z = m_init_z;
    float m_accum_qw = m_init_qw;
    float m_accum_qx = m_init_qx;
    float m_accum_qy = m_init_qy;
    float m_accum_qz = m_init_qz;

    // Previous key states for edge detection
    uint16_t m_prev_keys = 0;
    bool m_prev_ctrl = false;

    int m_stability_count = 0;
    bool key_event_processed = false;
    static constexpr int m_filter_threshold = 10;

    static constexpr float POS_RATE = 0.002f;
    static constexpr float ANG_RATE = 0.002f;
    static constexpr std::array<float, 3> MAX_POS = {0.4f, 0.4f, 0.8f};
    static constexpr float MAX_ANG = 3.14159f;

    // Low-pass filter coefficients
    static constexpr float ACCEL_ALPHA = 0.1f; // Accelerometer filter coefficient
    static constexpr float GYRO_ALPHA = 0.1f; // Gyroscope filter coefficient
    YandySlavePacket m_packet{};

    // Filtered IMU data storage
    struct FilteredImuData
    {
        float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
        float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    } m_filtered_imu;

    /**
     * @brief Apply first-order low-pass filter (exponential moving average)
     * @param new_value Current raw value
     * @param old_value Previous filtered value
     * @param alpha Filter coefficient (0.0-1.0, smaller = more smoothing)
     * @return Filtered value
     */
    static float lowPassFilter(float new_value, float old_value, float alpha)
    {
        return alpha * new_value + (1.0f - alpha) * old_value;
    }

    void processRemoteControl(const struct VT03RemotePacket& remote);
    YandyControlCmd processKeyboardCmd(const struct VT03RemotePacket& remote);
};

#endif // YANDYCOMMUNICATENODE_HPP
