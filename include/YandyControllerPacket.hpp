#ifndef RPL_YANDYCONTROLLERPACKET_HPP
#define RPL_YANDYCONTROLLERPACKET_HPP

struct __attribute__((packed)) YandyControllerPacket {
  float x;     // x
  float y;     // y
  float z;     // z
  float roll;  // roll
  float yaw;   // yaw
  float pitch; // pitch
};

#endif // RPL_YANDYCONTROLLERPACKET_HPP
