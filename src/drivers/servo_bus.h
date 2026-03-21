#pragma once
#include <Arduino.h>
#include <SCServo.h>

class ServoBus {
public:
  void begin(HardwareSerial& serial, uint32_t baud = 1000000UL);

  // Change baud rate of the underlying serial port (for scanning at different rates)
  void setBaud(uint32_t baud);

  bool ping(uint8_t id);
  int scan(uint8_t* ids, int maxIds, int& lastPingId);

  bool setPosition(uint8_t id, int pos, uint16_t speed, uint8_t acc);
  int readPosition(uint8_t id);
  bool torqueEnable(uint8_t id, bool en);

  bool readVoltage(uint8_t id, int& mv);
  bool readTemperature(uint8_t id, int& tempC);
  bool readStatus(uint8_t id, int& statusByte);  // reg 65: fault flags
  bool readLoad(uint8_t id, int& loadPct);        // reg 60-61: signed load %
  bool readCurrent(uint8_t id, int& currentMa);   // reg 69-70: motor current mA

  bool loadConfig(uint8_t id, uint8_t& outId, int& outMin, int& outMax,
                  int& outTorqueLimit, int& outCenterOffset,
                  int& outMode, int& outBaudIndex);
  bool saveId(uint8_t currentId, uint8_t newId);
  bool saveMinMax(uint8_t id, int minV, int maxV);
  bool saveTorqueLimit(uint8_t id, int limit);
  bool saveCenterOffset(uint8_t id, int offset);
  bool saveMode(uint8_t id, int mode);
  bool saveBaud(uint8_t id, int baudIndex);

private:
  SMS_STS _servo;
  HardwareSerial* _serial = nullptr;
  uint32_t _baud = 1000000UL;
};