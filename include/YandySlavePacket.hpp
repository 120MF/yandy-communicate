#ifndef RPL_YANDYSLAVEPACKET_HPP
#define RPL_YANDYSLAVEPACKET_HPP

#include <RPL/Meta/PacketTraits.hpp>
#include <cstdint>

enum class YandyControlCmd : uint8_t {
  CMD_NONE = 0x00, // 心跳/无操作

  // === 系统级指令 ===
  CMD_ERROR = 0x01,         // 急停 (Emergency Stop) - 最高优先级
  CMD_SWITCH_ENABLE = 0x02, // 启用/禁用输出 (Enable Toggle)
  CMD_RESET = 0x03,         // 复位 (Reset Error / Clear Accumulators)

  // === 模式切换指令 ===
  CMD_SWITCH_FETCH = 0x10, // 进入/退出 抓取模式 (Fetch Toggle)
  CMD_SWITCH_STORE = 0x11, // 进入/退出 存取矿模式 (Store Toggle)

  // === 手动操作指令 ===
  CMD_SWITCH_GRIP = 0x20, // 手动切换夹爪 (Gripper Toggle)

  // === 调试/修正指令 ===
  CMD_TOGGLE_HELD = 0x80, // 强制修改“持有矿石”状态
  CMD_INC_STORE = 0x81,   // 强制库存 +1
  CMD_DEC_STORE = 0x82    // 强制库存 -1
};

struct __attribute__((packed)) YandySlavePacket {
  float x;             // x
  float y;             // y
  float z;             // z
  float roll;          // roll
  float pitch;         // pitch
  float yaw;           // yaw
  float gimbal_z;      // gimbal_z
  float gimbal_yaw;    // gimbal_yaw
  float gimbal_pitch;  // gimbal_pitch
  YandyControlCmd cmd; // cmd
};

template <>
struct RPL::Meta::PacketTraits<YandySlavePacket>
    : PacketTraitsBase<PacketTraits<YandySlavePacket>> {
  static constexpr uint16_t cmd = 0x0604;
  static constexpr size_t size = sizeof(YandySlavePacket);
};
#endif // RPL_YANDYSLAVEPACKET_HPP
