//-- For Predator EYE --

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_RX 35
#define PIN_TX -1

#define PIN_OUT_CH7 25
#define PIN_OUT_CH9 26

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);

  crsf.begin(crsfSerial);

  pinMode(PIN_OUT_CH7, OUTPUT);
  pinMode(PIN_OUT_CH9, OUTPUT);
}

void loop() {
  crsf.update();

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    const crsf_channels_t* ch = crsf.getChannelsPacked();
    if (ch != nullptr) {

      // ----- GPIO control -----
      digitalWrite(PIN_OUT_CH7, (ch->ch7 > 1000) ? HIGH : LOW);
      digitalWrite(PIN_OUT_CH9, (ch->ch9 > 1000) ? HIGH : LOW);

      // ----- print channels -----
      Serial.printf(
        "\rCH0=%4d CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d CH9=%4d\n",
        ch->ch0, ch->ch1, ch->ch2, ch->ch3, ch->ch4,
        ch->ch5, ch->ch6, ch->ch7, ch->ch8, ch->ch9
      );
    }
  }
}
