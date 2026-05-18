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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum SignalType {
  NONE,
  PWM,
  PPM,
  SBUS_INV,
  SBUS_NON_INV
};

volatile SignalType detected_type = NONE;

volatile unsigned long last_edge_time = 0;
volatile unsigned long last_rising_time = 0;

volatile uint32_t last_cycle_time = 0;
volatile uint32_t last_high_duration = 0;

volatile uint16_t channels[16] = {0};

volatile uint8_t current_channel = 0;
volatile uint8_t pulse_counter = 0;

volatile bool signal_detected = false;

volatile uint16_t short_edge_counter = 0;
volatile uint32_t last_short_edge_time = 0;

volatile uint32_t valid_frame_count = 0;

HardwareSerial SBUS_Serial(2);

bool serial_active = false;
bool sbus_inversion_tested = false;

unsigned long sbus_start_time = 0;

float frequency = 0;

void IRAM_ATTR handleInterrupt() {

  uint32_t now = micros();

  uint32_t duration = now - last_edge_time;

  last_edge_time = now;

  bool pinState = GPIO.in & (1 << SIGNAL_PIN);

  signal_detected = true;

  // =========================================================
  // Detect UART-like fast edges (possible SBUS)
  // =========================================================

  // SBUS bit width is 10us. Broaden window for interrupt jitter.
  if (duration >= 2 && duration <= 30) {
    // If edges are close together, count them as part of a serial stream
    if (now - last_short_edge_time < 2000) {
      short_edge_counter++;
    } else {
      short_edge_counter = 1;
    }
    last_short_edge_time = now;

    // Strong evidence of SBUS stream
    if (short_edge_counter > 40) {
      detected_type = SBUS_NON_INV;
    }
  }

  // =========================================================
  // Rising edge
  // =========================================================

  if (pinState) {

    // Sync gap
    if (duration > 3000) {
      // Only try to detect PPM/PWM if we aren't already in SBUS mode
      if (detected_type == NONE || detected_type == PPM || detected_type == PWM) {
        if (pulse_counter >= 3 && pulse_counter <= 18) {
          detected_type = PPM;
        } else if (pulse_counter == 1 || (pulse_counter == 0 && detected_type == NONE)) {
          detected_type = PWM;
        }
      }

      pulse_counter = 0;
      current_channel = 0;
    }
    else {

      // PPM channel timing
      if (detected_type == PPM && current_channel < 12) {

        channels[current_channel] = now - last_rising_time;

        current_channel++;
      }
    }

    if (last_rising_time > 0) {
      last_cycle_time = now - last_rising_time;
    }

    last_rising_time = now;

    pulse_counter++;
  }

  // =========================================================
  // Falling edge
  // =========================================================

  else {

    last_high_duration = duration;

    // PWM pulse width
    if (detected_type == PWM) {

      channels[0] = duration;
    }

    // Initial PWM detection
    else if (detected_type == NONE &&
             duration > 900 &&
             duration < 2100 &&
             short_edge_counter < 5) {

      detected_type = PWM;
    }
  }
}

bool decodeSBUSFrame(uint8_t *buffer) {

  if (buffer[0] != 0x0F) {
    return false;
  }

  if ((buffer[24] & 0x0F) != 0x00) {
    return false;
  }

  channels[0]  = ((buffer[1]    | buffer[2] << 8) & 0x07FF);
  channels[1]  = ((buffer[2] >> 3 | buffer[3] << 5) & 0x07FF);
  channels[2]  = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF);
  channels[3]  = ((buffer[5] >> 1 | buffer[6] << 7) & 0x07FF);
  channels[4]  = ((buffer[6] >> 4 | buffer[7] << 4) & 0x07FF);
  channels[5]  = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF);
  channels[6]  = ((buffer[9] >> 2 | buffer[10] << 6) & 0x07FF);
  channels[7]  = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
  channels[8]  = ((buffer[12] | buffer[13] << 8) & 0x07FF);
  channels[9]  = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
  channels[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
  channels[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
  channels[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
  channels[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
  channels[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
  channels[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);

  return true;
}

void decodeSBUS() {

  while (SBUS_Serial.available() >= 25) {

    if (SBUS_Serial.peek() != 0x0F) {
      SBUS_Serial.read();
      continue;
    }

    uint8_t buffer[25];

    for (int i = 0; i < 25; i++) {
      buffer[i] = SBUS_Serial.read();
    }

    if (decodeSBUSFrame(buffer)) {

      valid_frame_count++;

      signal_detected = true;

      last_edge_time = micros();

      // Successfully decoded
      if (detected_type == SBUS_NON_INV ||
          detected_type == SBUS_INV) {
      }
    }
  }
}

void startSBUS(bool inverted) {

  SBUS_Serial.end();

  delay(20);

  SBUS_Serial.begin(
    100000,
    SERIAL_8E2,
    SIGNAL_PIN,
    -1,
    inverted
  );

  detected_type = inverted ? SBUS_INV : SBUS_NON_INV;

  serial_active = true;

  sbus_start_time = millis();

  valid_frame_count = 0;
}

void stopSBUS() {

  SBUS_Serial.end();

  serial_active = false;

  detected_type = NONE;

  short_edge_counter = 0;

  valid_frame_count = 0;

  sbus_inversion_tested = false;

  attachInterrupt(
    digitalPinToInterrupt(SIGNAL_PIN),
    handleInterrupt,
    CHANGE
  );
}

void setup() {

  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {

    Serial.println("SSD1306 allocation failed");

    while (true);
  }

  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(WHITE);

  pinMode(SIGNAL_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(SIGNAL_PIN),
    handleInterrupt,
    CHANGE
  );
}

void loop() {

  // =========================================================
  // Enter SBUS mode
  // =========================================================

  if (detected_type == SBUS_NON_INV &&
      !serial_active) {

    detachInterrupt(digitalPinToInterrupt(SIGNAL_PIN));

    startSBUS(false);
  }

  // =========================================================
  // Decode SBUS
  // =========================================================

  if (serial_active) {

    decodeSBUS();

    // No valid packets?
    // Retry with inversion enabled
    if (!sbus_inversion_tested &&
        valid_frame_count == 0 &&
        millis() - sbus_start_time > 150) {

      sbus_inversion_tested = true;

      startSBUS(true);
    }

    // Still nothing -> give up
    if (valid_frame_count == 0 &&
        millis() - sbus_start_time > 400) {

      stopSBUS();
    }
  }

  // =========================================================
  // Signal timeout
  // =========================================================

  if (micros() - last_edge_time > 500000) {

    detected_type = NONE;

    pulse_counter = 0;

    short_edge_counter = 0;

    current_channel = 0;

    if (serial_active) {
      stopSBUS();
    }
  }

  // =========================================================
  // OLED update
  // =========================================================

  static uint32_t last_update = 0;

  if (millis() - last_update > 500) {

    last_update = millis();

    display.clearDisplay();

    display.setCursor(0, 0);

    if (!signal_detected ||
        micros() - last_edge_time > 200000) {

      signal_detected = false;
      display.println("No Signal");
    }
    else {

      display.print("Type: ");

      switch (detected_type) {

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

        case SBUS_NON_INV:

          display.println("SBUS");

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
          display.print(valid_frame_count);

          break;

        case SBUS_INV:

          display.println("SBUS INV");

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
          display.print(valid_frame_count);

          break;

        default:

          display.println("Scanning...");

          break;
      }
    }

    display.display();
  }
}
