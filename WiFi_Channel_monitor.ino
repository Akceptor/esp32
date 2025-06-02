#include <WiFi.h>

int rssi[14]; // index 1–13 used

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // ensure no connection

}

void loop() {
  for (int i = 1; i <= 13; i++) {
    rssi[i] = -100;
  }

  int n = WiFi.scanNetworks(false, true); // async = false, show hidden = true

  for (int i = 0; i < n; ++i) {
    int ch = WiFi.channel(i);
    int signal = WiFi.RSSI(i);
    if (ch >= 1 && ch <= 13 && signal > rssi[ch]) {
      rssi[ch] = signal; 
    }
  }

  for (int i = 1; i <= 13; i++) {
    int signal = rssi[i];
    int barLength = map(signal, -100, -30, 0, 20); // up to 20 chars wide
    if (barLength < 0) barLength = 0;

    Serial.printf("   %2d    | %4d dBm | ", i, signal);
    for (int j = 0; j < barLength; j++) Serial.print("█");
    Serial.println();
  }

  delay(200);
}
