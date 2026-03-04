#pragma once
#include <Arduino.h>

struct Packet {
  uint8_t type;
  uint8_t len;
  uint8_t payload[255];
};

class Protocol {
public:
  void begin(HardwareSerial &serial);

  bool poll(Packet &out); // true when a valid full packet is decoded
  void send(uint8_t type, const uint8_t *payload, uint8_t len);
  void sendAck(uint8_t cmdType, uint8_t status);

  static const uint8_t SOF0 = 0xAA;
  static const uint8_t SOF1 = 0x55;

  enum MsgType : uint8_t {
    CMD_SET_PWM   = 0x01, // payload: int16 L, int16 R
    CMD_ESTOP     = 0x02, // payload: uint8 0/1
    CMD_PING      = 0x03,

    MSG_ENCODERS  = 0x81, // payload: int32 L, int32 R, uint16 dt_ms
    MSG_ACK       = 0x90, // payload: uint8 cmdType, uint8 status
  };

private:
  HardwareSerial *_s = nullptr;

  enum ParseState : uint8_t { WAIT_SOF0, WAIT_SOF1, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC };
  ParseState _ps = WAIT_SOF0;

  uint8_t _rxType = 0;
  uint8_t _rxLen = 0;
  uint8_t _rxIdx = 0;
  uint8_t _rxPayload[255];

  static uint8_t crc8_update(uint8_t crc, uint8_t data);
  static uint8_t crc8_compute(const uint8_t *data, size_t n);
};