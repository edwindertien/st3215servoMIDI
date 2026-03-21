#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>

#include "config.h"
#include "app_state.h"

#include "drivers/encoder_unit.h"
#include "drivers/oled_ui.h"
#include "drivers/servo_bus.h"
#include "drivers/midi_engine.h"
#include "drivers/persist.h"
#include "app/app.h"

AppState    appState;
UnitEncoder encoder(Wire1, HW::ENC_ADDR);
OledUi      ui(HW::OLED_W, HW::OLED_H, &Wire, HW::OLED_ADDR);
ServoBus    bus;
App         app(appState, encoder, ui, bus, midiEngine);

static void onCCReceived(uint8_t channel, uint8_t cc, uint8_t value) {
  app.onMidiCC(channel, cc, value);
}

static void onAnyReceived(MidiMsgType type, uint8_t channel,
                           uint8_t byte1, uint8_t byte2, int16_t int14) {
  app.onMidiAny(type, channel, byte1, byte2, int14);
}

void setup() {
  // -----------------------------------------------------------------------
  // USB MIDI must be initialised first — before any other peripheral.
  // TinyUSB on RP2040 needs the device descriptor registered before the
  // USB stack starts (which happens implicitly on first delay/yield).
  // -----------------------------------------------------------------------
  midiEngine.setOnCC(onCCReceived);
  midiEngine.setOnAny(onAnyReceived);
  midiEngine.begin();

  // Brief wait for USB enumeration before touching other peripherals
  delay(200);

  // Wait up to 1.5 s for the host to mount the MIDI device.
  // Non-blocking — the rest of setup continues if USB isn't connected.
  {
    unsigned long t = millis();
    while (!midiEngine.isMounted() && millis() - t < 1500) delay(10);
  }

  // OLED on I2C0
  Wire.setSDA(HW::OLED_SDA_PIN);
  Wire.setSCL(HW::OLED_SCL_PIN);
  Wire.begin();

  // Servo UART
  Serial1.setTX(HW::SERVO_TX_PIN);
  Serial1.setRX(HW::SERVO_RX_PIN);
  Serial1.begin(HW::SERVO_BAUD);

  appState.oledOk = ui.begin();
  if (appState.oledOk) ui.splash("ST3215 Tool", "Init...");

  appState.encoderOk = encoder.begin(HW::ENC_SDA_PIN, HW::ENC_SCL_PIN, HW::ENC_I2C_HZ);
  if (appState.encoderOk) encoder.setLedColor(0, 0x002000);

  bus.begin(Serial1);

  // Mount LittleFS — after USB is up to avoid DMA timing conflicts
  persist.begin();

  if (appState.oledOk) ui.splash("Starting...");
  app.begin();

  delay(300);
}

void loop() {
  app.tick();
}