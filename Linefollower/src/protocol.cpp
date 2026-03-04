#include "protocol.h"
#include <string.h>

void Protocol::begin(HardwareSerial &serial) { _s = &serial; }

uint8_t Protocol::crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) crc = (crc << 1) ^ 0x07;
    else crc <<= 1;
  }
  return crc;
}

uint8_t Protocol::crc8_compute(const uint8_t *data, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) crc = crc8_update(crc, data[i]);
  return crc;
}

bool Protocol::poll(Packet &out) {
  if (!_s) return false;

  while (_s->available() > 0) {
    uint8_t b = (uint8_t)_s->read();

    switch (_ps) {
      case WAIT_SOF0:
        if (b == SOF0) _ps = WAIT_SOF1;
        break;

      case WAIT_SOF1:
        _ps = (b == SOF1) ? WAIT_TYPE : WAIT_SOF0;
        break;

      case WAIT_TYPE:
        _rxType = b;
        _ps = WAIT_LEN;
        break;

      case WAIT_LEN:
        _rxLen = b;
        _rxIdx = 0;
        if (_rxLen == 0) _ps = WAIT_CRC;
        else if (_rxLen <= sizeof(_rxPayload)) _ps = WAIT_PAYLOAD;
        else _ps = WAIT_SOF0; // invalid length, resync
        break;

      case WAIT_PAYLOAD:
        _rxPayload[_rxIdx++] = b;
        if (_rxIdx >= _rxLen) _ps = WAIT_CRC;
        break;

      case WAIT_CRC: {
        uint8_t crcData[2 + 255];
        crcData[0] = _rxType;
        crcData[1] = _rxLen;
        if (_rxLen > 0) memcpy(&crcData[2], _rxPayload, _rxLen);
        uint8_t crc = crc8_compute(crcData, 2 + _rxLen);

        _ps = WAIT_SOF0;

        if (crc != b) return false;

        out.type = _rxType;
        out.len = _rxLen;
        if (_rxLen > 0) memcpy(out.payload, _rxPayload, _rxLen);
        return true;
      }
    }
  }
  return false;
}

void Protocol::send(uint8_t type, const uint8_t *payload, uint8_t len) {
  if (!_s) return;

  uint8_t hdr[4] = { SOF0, SOF1, type, len };

  uint8_t crcData[2 + 255];
  crcData[0] = type;
  crcData[1] = len;
  if (len > 0 && payload) memcpy(&crcData[2], payload, len);
  uint8_t crc = crc8_compute(crcData, 2 + len);

  _s->write(hdr, sizeof(hdr));
  if (len > 0 && payload) _s->write(payload, len);
  _s->write(crc);
}

// send acknowldgement for a received command
void Protocol::sendAck(uint8_t cmdType, uint8_t status) {
  uint8_t payload[2] = { cmdType, status };
  send(MSG_ACK, payload, (uint8_t)sizeof(payload));
}