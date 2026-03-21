#include "servo_bus.h"

void ServoBus::begin(HardwareSerial& serial, uint32_t baud) {
  _serial = &serial;
  _baud = baud;
  _servo.pSerial = &serial;
  _servo.IOTimeOut = 20;
}

void ServoBus::setBaud(uint32_t baud) {
  if (!_serial) return;
  _baud = baud;
  _serial->begin(baud);
  delay(5); // let serial settle
}

bool ServoBus::ping(uint8_t id) {
  int r = _servo.Ping(id);
  return r == id;
}

int ServoBus::scan(uint8_t* ids, int maxIds, int& lastPingId) {
  int count = 0;
  for (int id = 0; id <= 253; ++id) {
    lastPingId = id;
    if (ping((uint8_t)id) && count < maxIds) {
      ids[count++] = (uint8_t)id;
    }
    delay(2);
  }
  return count;
}

bool ServoBus::setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc) {
  return _servo.WritePosEx(id, pos, speed, acc) >= 0;
}

int ServoBus::readPosition(uint8_t id) {
  int v = _servo.ReadPos(id);
  return (v >= 0 && v <= 4095) ? v : -1;
}

bool ServoBus::torqueEnable(uint8_t id, bool en) {
  return _servo.EnableTorque(id, en ? 1 : 0) >= 0;
}

bool ServoBus::readVoltage(uint8_t id, int& mv) {
  int v = _servo.ReadVoltage(id);
  if (v < 0) return false;
  mv = v;
  return true;
}

bool ServoBus::readTemperature(uint8_t id, int& tempC) {
  int v = _servo.ReadTemper(id);
  if (v < 0) return false;
  tempC = v;
  return true;
}

bool ServoBus::readStatus(uint8_t id, int& statusByte) {
  // Register 65 (0x41) — present status / fault flags
  int v = _servo.readByte(id, 65);
  if (v < 0) return false;
  statusByte = v;
  return true;
}

bool ServoBus::readLoad(uint8_t id, int& loadPct) {
  // Registers 60-61 — present load (signed 11-bit: bit15=direction, bits0-10=magnitude 0..1000)
  int v = _servo.readWord(id, SMS_STS_PRESENT_LOAD_L);
  if (v < 0) return false;
  // Bit 10 is the sign bit in the STS protocol
  int magnitude = v & 0x3FF;
  int sign      = (v >> 10) & 1;
  // Scale 0..1000 → 0..100%
  loadPct = (sign ? -1 : 1) * (magnitude * 100 / 1000);
  return true;
}

bool ServoBus::readCurrent(uint8_t id, int& currentMa) {
  // Registers 69-70 — present current (raw units, 1 LSB ≈ 6.5 mA per Feetech datasheet)
  int v = _servo.readWord(id, SMS_STS_PRESENT_CURRENT_L);
  if (v < 0) return false;
  currentMa = v * 65 / 10; // approx 6.5 mA per unit
  return true;
}

bool ServoBus::loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                          int& outTorqueLimit, int& outCenterOffset,
                          int& outMode, int& outBaudIndex) {
  if (!ping(id)) return false;

  outId = id;

  int minV        = _servo.readWord(id, SMS_STS_MIN_ANGLE_LIMIT_L);
  int maxV        = _servo.readWord(id, SMS_STS_MAX_ANGLE_LIMIT_L);
  int torqueLimit = _servo.readWord(id, SMS_STS_TORQUE_LIMIT_L);
  int centerOfs   = _servo.readWord(id, SMS_STS_OFS_L);
  int mode        = _servo.readByte(id, SMS_STS_MODE);
  int baudIdx     = _servo.readByte(id, SMS_STS_BAUD_RATE);

  if (minV < 0 || maxV < 0) return false;

  outMin          = minV;
  outMax          = maxV;
  outTorqueLimit  = (torqueLimit >= 0) ? torqueLimit  : 1000;
  outMode         = (mode        >= 0) ? (mode & 0x03) : 0;
  outBaudIndex    = (baudIdx     >= 0 && baudIdx < 8) ? baudIdx : 0;

  // Center offset is a signed value stored as two's-complement in 12-bit space
  if (centerOfs >= 0) {
    if (centerOfs > 2047) centerOfs -= 4096;
    outCenterOffset = centerOfs;
  } else {
    outCenterOffset = 0;
  }
  return true;
}

bool ServoBus::saveId(uint8_t currentId, uint8_t newId) {
  _servo.unLockEprom(currentId);
  int ok = _servo.writeByte(currentId, SMS_STS_ID, newId);
  _servo.LockEprom(currentId);
  return ok >= 0;
}

bool ServoBus::saveMinMax(uint8_t id, int minV, int maxV) {
  _servo.unLockEprom(id);
  int a = _servo.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, minV);
  int b = _servo.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, maxV);
  _servo.LockEprom(id);
  return (a >= 0 && b >= 0);
}

bool ServoBus::saveTorqueLimit(uint8_t id, int limit) {
  _servo.unLockEprom(id);
  int ok = _servo.writeWord(id, SMS_STS_TORQUE_LIMIT_L, (uint16_t)limit);
  _servo.LockEprom(id);
  return ok >= 0;
}

bool ServoBus::saveCenterOffset(uint8_t id, int offset) {
  uint16_t wire = (offset >= 0) ? (uint16_t)offset : (uint16_t)(offset + 4096);
  _servo.unLockEprom(id);
  int ok = _servo.writeWord(id, SMS_STS_OFS_L, wire);
  _servo.LockEprom(id);
  return ok >= 0;
}

bool ServoBus::saveMode(uint8_t id, int mode) {
  _servo.unLockEprom(id);
  int ok = _servo.writeByte(id, SMS_STS_MODE, (uint8_t)(mode & 0x03));
  _servo.LockEprom(id);
  return ok >= 0;
}

bool ServoBus::saveBaud(uint8_t id, int baudIndex) {
  if (baudIndex < 0 || baudIndex >= 8) return false;
  _servo.unLockEprom(id);
  int ok = _servo.writeByte(id, SMS_STS_BAUD_RATE, (uint8_t)baudIndex);
  _servo.LockEprom(id);
  return ok >= 0;
}