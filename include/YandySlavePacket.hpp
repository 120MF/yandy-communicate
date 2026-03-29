#ifndef RPL_YANDYSLAVEPACKET_HPP
#define RPL_YANDYSLAVEPACKET_HPP

#include <RPL/Meta/PacketTraits.hpp>
#include <cstdint>

enum class YandyControlCmd : uint8_t
{
  CMD_NONE = 0x00, // 心跳/无操作

  // === 系统级指令 ===
  CMD_ERROR = 0x01, // 急停 (Emergency Stop) - 最高优先级
  CMD_SWITCH_ENABLE = 0x02, // 启用/禁用输出 (Enable Toggle)
  CMD_RESET = 0x03, // 复位 (Reset Error / Clear Accumulators)

  // === 模式切换指令 ===
  CMD_SWITCH_FETCH = 0x10, // 进入/退出 抓取模式 (Fetch Toggle)
  CMD_SWITCH_STORE = 0x11, // 进入/退出 存取矿模式 (Store Toggle)

  // === 手动操作指令 ===
  CMD_SWITCH_GRIP = 0x20, // 手动切换夹爪 (Gripper Toggle)

  // === 调试/修正指令 ===
  CMD_TOGGLE_HELD = 0x80, // 强制修改“持有矿石”状态
  CMD_INC_STORE = 0x81, // 强制库存 +1
  CMD_DEC_STORE = 0x82 // 强制库存 -1
};

struct __attribute__((packed)) YandySlavePacket
{
  float x; // x
  float y; // y
  float z; // z
  float qw; // qw
  float qx; // qx
  float qy; // qy
  float qz; // qz
  float gimbal_z; // gimbal_z
  float gimbal_yaw; // gimbal_yaw
  float gimbal_pitch; // gimbal_pitch

  // 底盘加速度计数据 (底盘 body frame, 单位 m/s²)
  // 静止水平时：ax≈0, ay≈0, az≈+9.81 (Z 轴朝上)
  float base_ax; // X 方向线性比力
  float base_ay; // Y 方向线性比力
  float base_az; // Z 方向线性比力

  // 底盘角速度 (底盘 body frame, 单位 rad/s)
  float base_gx; // 绕 X 轴角速度 (roll rate)
  float base_gy; // 绕 Y 轴角速度 (pitch rate)
  float base_gz; // 绕 Z 轴角速度 (yaw rate)

  // 底盘姿态四元数 (传感器融合估计，world → chassis)
  float base_qw; // 四元数 W 分量 (实部)
  float base_qx; // 四元数 X 分量
  float base_qy; // 四元数 Y 分量
  float base_qz; // 四元数 Z 分量

  YandyControlCmd cmd; // cmd
};

template <>
struct RPL::Meta::PacketTraits<YandySlavePacket>
  : PacketTraitsBase<PacketTraits<YandySlavePacket>>
{
  static constexpr uint16_t cmd = 0x0604;
  static constexpr size_t size = sizeof(YandySlavePacket);
};
#endif // RPL_YANDYSLAVEPACKET_HPP
