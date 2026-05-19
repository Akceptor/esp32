#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/gpio_struct.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

#define OLED_SDA 21
#define OLED_SCL 22

#define SIGNAL_PIN 25
#define BUTTON_PIN 0

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum SignalType {
  NONE,
  PWM,
  PPM,
  SBUS_INV,
  SBUS_NON_INV,
  CRSF,
  CRSF_INV
};

volatile SignalType detected_type = NONE;

volatile unsigned long last_edge_time = 0;
volatile unsigned long last_rising_time = 0;

volatile uint32_t last_cycle_time = 0;
volatile uint32_t last_high_duration = 0;

volatile uint16_t channels[8] = {0};

volatile uint8_t current_channel = 0;
volatile uint8_t pulse_counter = 0;

volatile bool signal_detected = false;

volatile uint32_t sbus_valid_frame_count = 0;
volatile uint32_t crsf_valid_frame_count = 0;

HardwareSerial SerialRX(2);

bool serial_active = false;

float frequency = 0;

// =========================================================
// CRSF
// =========================================================

#define CRSF_BAUD 420000

#define CRSF_ADDR_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDR_RECEIVER          0xEC
#define CRSF_ADDR_TRANSMITTER       0xEA

#define CRSF_FRAME_RC_CHANNELS_PACKED 0x16
#define CRSF_RC_CHANNEL_PAYLOAD_SIZE 22
#define CRSF_RC_FRAME_LENGTH 24

// =========================================================
// Interrupt
// =========================================================

void IRAM_ATTR handleInterrupt() {

  uint32_t now = micros();
  uint32_t duration = now - last_edge_time;

  last_edge_time = now;

  bool pinState = GPIO.in & (1 << SIGNAL_PIN);

  signal_detected = true;

  // =========================================================
  // Rising edge
  // =========================================================

  if (pinState) {

    // Sync gap

    if (duration > 2500) {

      pulse_counter = 0;
      current_channel = 0;
    }
    else {

      if (detected_type == PPM &&
          current_channel < 8 &&
          last_rising_time > 0) {

        channels[current_channel] =
            now - last_rising_time;

        current_channel++;
      }
    }

    if (last_rising_time > 0) {

      last_cycle_time =
          now - last_rising_time;
    }

    last_rising_time = now;

    pulse_counter++;
  }

  // =========================================================
  // Falling edge
  // =========================================================

  else {

    last_high_duration = duration;

    if (detected_type == PWM) {

      channels[0] = duration;
    }
  }
}

// =========================================================
// Shared unpacking
// =========================================================

bool unpackChannels(uint8_t *buffer) {

  channels[0]  = ((buffer[0]       | buffer[1] << 8) & 0x07FF);
  channels[1]  = ((buffer[1] >> 3  | buffer[2] << 5) & 0x07FF);
  channels[2]  = ((buffer[2] >> 6  | buffer[3] << 2 | buffer[4] << 10) & 0x07FF);
  channels[3]  = ((buffer[4] >> 1  | buffer[5] << 7) & 0x07FF);
  channels[4]  = ((buffer[5] >> 4  | buffer[6] << 4) & 0x07FF);
  channels[5]  = ((buffer[6] >> 7  | buffer[7] << 1 | buffer[8] << 9) & 0x07FF);
  channels[6]  = ((buffer[8] >> 2  | buffer[9] << 6) & 0x07FF);
  channels[7]  = ((buffer[9] >> 5  | buffer[10] << 3) & 0x07FF);

  return true;
}

// =========================================================
// SBUS
// =========================================================

bool decodeSBUSFrame(uint8_t *buffer) {

  if (buffer[0] != 0x0F) {
    return false;
  }

  if ((buffer[24] & 0x0F) != 0x00) {
    return false;
  }

  return unpackChannels(&buffer[1]);
}

void decodeSBUS() {

  while (SerialRX.available() >= 25) {

    if (SerialRX.peek() != 0x0F) {

      SerialRX.read();
      continue;
    }

    uint8_t buffer[25];

    if (SerialRX.readBytes(buffer, 25) != 25) {
      return;
    }

    if (decodeSBUSFrame(buffer)) {

      sbus_valid_frame_count++;

      signal_detected = true;

      last_edge_time = micros();
    }
  }
}

// =========================================================
// CRSF CRC8 DVB-S2
// =========================================================

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {

  uint8_t crc = 0;

  while (len--) {

    crc ^= *ptr++;

    for (uint8_t i = 0; i < 8; i++) {

      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      }
      else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

// =========================================================
// CRSF
// =========================================================

void decodeCRSF() {

  while (SerialRX.available() >= 4) {

    uint8_t addr = SerialRX.peek();

    if (
      addr != CRSF_ADDR_FLIGHT_CONTROLLER &&
      addr != CRSF_ADDR_RECEIVER &&
      addr != CRSF_ADDR_TRANSMITTER
    ) {

      SerialRX.read();
      continue;
    }

    // Address

    uint8_t device_addr = SerialRX.read();

    // Length

    if (SerialRX.available() < 1) {
      return;
    }

    uint8_t frame_length = SerialRX.read();

    if (frame_length < 2 || frame_length > 62) {
      continue;
    }

    if (SerialRX.available() < frame_length) {
      return;
    }

    uint8_t frame_buffer[64];

    if (SerialRX.readBytes(frame_buffer, frame_length) != frame_length) {
      return;
    }

    uint8_t frame_type =
        frame_buffer[0];

    uint8_t received_crc =
        frame_buffer[frame_length - 1];

    // CRC over Type + Payload

    uint8_t calculated_crc =
        crsf_crc8(frame_buffer, frame_length - 1);

    if (calculated_crc != received_crc) {
      continue;
    }

    // RC Channels

    if (
      frame_type == CRSF_FRAME_RC_CHANNELS_PACKED &&
      frame_length == CRSF_RC_FRAME_LENGTH
    ) {

      unpackChannels(&frame_buffer[1]);

      crsf_valid_frame_count++;

      signal_detected = true;

      last_edge_time = micros();
    }
  }
}

// =========================================================
// Start/Stop Serial
// =========================================================

void startSBUS(bool inverted) {

  SerialRX.end();

  detachInterrupt(digitalPinToInterrupt(SIGNAL_PIN));

  delay(20);

  SerialRX.begin(
    100000,
    SERIAL_8E2,
    SIGNAL_PIN,
    -1,
    inverted
  );

  detected_type =
      inverted
      ? SBUS_INV
      : SBUS_NON_INV;

  serial_active = true;

  sbus_valid_frame_count = 0;
}

void startCRSF(bool inverted) {

  SerialRX.end();

  detachInterrupt(digitalPinToInterrupt(SIGNAL_PIN));

  delay(20);

  SerialRX.begin(
    CRSF_BAUD,
    SERIAL_8N1,
    SIGNAL_PIN,
    -1,
    inverted
  );

  detected_type =
      inverted
      ? CRSF_INV
      : CRSF;

  serial_active = true;

  crsf_valid_frame_count = 0;
}

void stopSerialMode() {

  SerialRX.end();

  serial_active = false;

  sbus_valid_frame_count = 0;
  crsf_valid_frame_count = 0;

  attachInterrupt(
    digitalPinToInterrupt(SIGNAL_PIN),
    handleInterrupt,
    CHANGE
  );
}

// =========================================================
// Mode cycle
// =========================================================

void cycleMode() {

  if (serial_active) {
    stopSerialMode();
  }

  if (detected_type == PWM) {

    detected_type = PPM;
  }
  else if (detected_type == PPM) {

    startSBUS(false);
  }
  else if (detected_type == SBUS_NON_INV) {

    startSBUS(true);
  }
  else if (detected_type == SBUS_INV) {

    startCRSF(false);
  }
  else if (detected_type == CRSF) {

    startCRSF(true);
  }
  else if (detected_type == CRSF_INV) {

    detected_type = PWM;
  }
  else {

    detected_type = PWM;
  }
}

// =========================================================
// Setup
// =========================================================

void setup() {

  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {

    while (true);
  }

  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(WHITE);

  pinMode(SIGNAL_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  detected_type = PWM;

  attachInterrupt(
    digitalPinToInterrupt(SIGNAL_PIN),
    handleInterrupt,
    CHANGE
  );
}

// =========================================================
// Loop
// =========================================================

void loop() {

  // =========================================================
  // Button
  // =========================================================

  static bool lastButtonState = HIGH;

  bool currentButtonState =
      digitalRead(BUTTON_PIN);

  if (
    lastButtonState == HIGH &&
    currentButtonState == LOW
  ) {

    cycleMode();

    delay(150);
  }

  lastButtonState = currentButtonState;

  // =========================================================
  // Decode serial protocols
  // =========================================================

  if (serial_active) {

    if (
      detected_type == SBUS_NON_INV ||
      detected_type == SBUS_INV
    ) {

      decodeSBUS();
    }
    else if (
      detected_type == CRSF ||
      detected_type == CRSF_INV
    ) {

      decodeCRSF();
    }
  }

  // =========================================================
  // Timeout
  // =========================================================

  if (micros() - last_edge_time > 500000) {

    pulse_counter = 0;

    current_channel = 0;
  }

  // =========================================================
  // OLED
  // =========================================================

  static uint32_t last_update = 0;

  if (millis() - last_update > 500) {

    last_update = millis();

    display.clearDisplay();

    display.setCursor(0, 0);

    bool validSignal =
        signal_detected &&
        (micros() - last_edge_time < 200000);

    if (!validSignal) {

      display.println("No Signal");

      display.print("Type: ");

      switch (detected_type) {

        case PWM:
          display.println("PWM");
          break;

        case PPM:
          display.println("PPM");
          break;

        case SBUS_NON_INV:
          display.println("SBUS");
          break;

        case SBUS_INV:
          display.println("SBUS INV");
          break;

        case CRSF:
          display.println("CRSF");
          break;

        case CRSF_INV:
          display.println("CRSF INV");
          break;

        default:
          display.println("NONE");
          break;
      }
    }
    else {

      display.print("Type: ");

      switch (detected_type) {

        // =====================================================
        // PWM
        // =====================================================

        case PWM:

          display.println("PWM");

          frequency =
            (last_cycle_time > 0)
            ? 1000000.0 / last_cycle_time
            : 0;

          display.print("Freq: ");
          display.print(frequency, 1);
          display.println(" Hz");

          display.print("Width: ");
          display.print(last_high_duration);
          display.println(" us");

          break;

        // =====================================================
        // PPM
        // =====================================================

        case PPM:

          display.println("PPM");

          for (int i = 0; i < 8; i++) {

            int col = (i % 2) * 64;
            int row = 16 + (i / 2) * 10;

            display.setCursor(col, row);

            display.print("CH");
            display.print(i + 1);
            display.print(":");
            display.print(channels[i]);
          }

          break;

        // =====================================================
        // SBUS
        // =====================================================

        case SBUS_NON_INV:
        case SBUS_INV:

          display.println(
            detected_type == SBUS_INV
            ? "SBUS INV"
            : "SBUS"
          );

          for (int i = 0; i < 8; i++) {

            int col = (i % 2) * 64;
            int row = 16 + (i / 2) * 10;

            display.setCursor(col, row);

            display.print("C");
            display.print(i + 1);
            display.print(":");
            display.print(channels[i]);
          }

          display.setCursor(0, 56);

          display.print("PKT:");
          display.print(sbus_valid_frame_count);

          break;

        // =====================================================
        // CRSF
        // =====================================================

        case CRSF:
        case CRSF_INV:

          display.println(
            detected_type == CRSF_INV
            ? "CRSF INV"
            : "CRSF"
          );

          for (int i = 0; i < 8; i++) {

            int col = (i % 2) * 64;
            int row = 16 + (i / 2) * 10;

            display.setCursor(col, row);

            display.print("C");
            display.print(i + 1);
            display.print(":");
            display.print(channels[i]);
          }

          display.setCursor(0, 56);

          display.print("PKT:");
          display.print(crsf_valid_frame_count);

          break;

        default:

          display.println("Unknown");

          break;
      }
    }

    display.display();
  }
}
