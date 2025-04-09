/*
 * ESP32 + CC1101 RF Activity Scanner
 * Detects RF transmission in a specified frequency range without decoding.
 * Ideal for tools like ELRS signal presence detection or analog sniffing.
 */

#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// CC1101 pins for ESP32
#define CC1101_SCK  18
#define CC1101_MISO 19
#define CC1101_MOSI 23
#define CC1101_CS   5
#define CC1101_GDO0 2

// Settings
#define START_FREQ   905.0     // MHz - starting frequency
#define END_FREQ     925.0     // MHz - ending frequency
#define STEP_FREQ    1       // MHz step
#define RSSI_THRESHOLD -90     // dBm threshold to detect activity
#define SCAN_DELAY   10       // ms delay between scans

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\nESP32 + CC1101 RF Scanner");
  Serial.println("----------------------------");

  // Start SPI and initialize CC1101
  SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
  ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
  ELECHOUSE_cc1101.Init();

  // Receiver mode, no sync required, wide-open filtering
  ELECHOUSE_cc1101.setCCMode(1);     // 1 = Receiver mode
  ELECHOUSE_cc1101.setModulation(0); // 0 = 2-FSK
  ELECHOUSE_cc1101.setPA(10);        // Set power amplifier level
  ELECHOUSE_cc1101.setRxBW(101);      // Wider RX bandwidth (default 58 kHz)
  ELECHOUSE_cc1101.setSyncMode(7);   // Sync off (good for raw detection)
  ELECHOUSE_cc1101.setAdrChk(0);     // Disable address check

  Serial.println("Scanner ready!");
}

int count = (END_FREQ-START_FREQ)/STEP_FREQ;

void loop() {
  int rssis = 0;
  for (float freq = START_FREQ; freq <= END_FREQ; freq += STEP_FREQ) {
    ELECHOUSE_cc1101.setMHZ(freq);
    ELECHOUSE_cc1101.SetRx(); // Start receiver
    delay(SCAN_DELAY);

    int rssi = ELECHOUSE_cc1101.getRssi();
    rssis = rssis + rssi;
  }
    int rssi = rssis/count;
    int barLength = map(abs(rssi), 120, 50, 0, 30); // Adjust for RSSI range
    barLength = constrain(barLength, 0, 30);

    String bar = "";
    for (int i = 0; i < barLength; i++) {
      bar += "|";
    }
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm ");
    Serial.println(bar);
}
