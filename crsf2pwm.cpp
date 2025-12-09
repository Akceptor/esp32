#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_RX_IN1 18  // Yellow, 433
#define PIN_TX_IN1 19

#define PIN_RX_IN2 25  // Green, 2.4G
#define PIN_TX_IN2 26

#define PIN_RX_OUT 16
#define PIN_TX_OUT 17

int aileronsPin = 12;
int elevatorPin = 13;
int throttlePin = 14;
int rudderPin = 15;
int bimbaPin = 21;

int aileronsPWMChannel = 1;
int elevatorPWMChannel = 2;
int throttlePWMChannel = 3;
int rudderPWMChannel = 5;
int bimbaPWMChannel = 6;

HardwareSerial crsfSerialIn1(1);
AlfredoCRSF crsfIn1;

HardwareSerial crsfSerialIn2(2);
AlfredoCRSF crsfIn2;

HardwareSerial crsfSerialOut(0);
AlfredoCRSF crsfOut;

// Встановлюємо duty cycle так, щоб максимальний відсоток (100%) давав бажане значення
void SetServoPos(float percent, int pwmChannel)
{
    // Кориговані значення
    const uint32_t minDuty = 5000;  // Мінімум
    const uint32_t midDuty = 6353;  // 9,7% (центр)
    const uint32_t maxDuty = 7150;  // Тепер 12,7% (максимум)

    uint32_t duty = 0;

    if (percent <= 50) {
        duty = minDuty + ((percent / 50.0) * (midDuty - minDuty)); // для нижньої половини
    } else {
        duty = midDuty + (((percent - 50) / 50.0) * (maxDuty - midDuty)); // для верхньої половини
    }

    ledcWrite(pwmChannel, duty);
}

void setup() {
  crsfSerialIn1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_IN1, PIN_TX_IN1);
  delay(1000);
  crsfSerialIn2.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_IN2, PIN_TX_IN2);
  delay(1000);
  crsfSerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_OUT, PIN_TX_OUT);
  delay(1000);
  crsfIn1.begin(crsfSerialIn1);
  crsfIn2.begin(crsfSerialIn2);
  crsfOut.begin(crsfSerialOut);

  // Налаштування пінів для PWM на ESP32 з 16-бітною розрядністю
  ledcSetup(aileronsPWMChannel, 50, 16);  // 50 Гц, 16 біт розрядності
  ledcSetup(elevatorPWMChannel, 50, 16);
  ledcSetup(throttlePWMChannel, 50, 16);
  ledcSetup(rudderPWMChannel, 50, 16);
  ledcSetup(bimbaPWMChannel, 50, 16);

  // Підключення пінів до PWM каналу
  ledcAttachPin(aileronsPin, aileronsPWMChannel);
  ledcAttachPin(elevatorPin, elevatorPWMChannel);
  ledcAttachPin(throttlePin, throttlePWMChannel);
  ledcAttachPin(rudderPin, rudderPWMChannel);
  ledcAttachPin(bimbaPin, bimbaPWMChannel);
}

int getLinkQuality(AlfredoCRSF& crsf) {
  const crsfLinkStatistics_t* stat_ptr = crsf.getLinkStatistics();
  return stat_ptr->uplink_Link_quality;
}

void sendChannels(AlfredoCRSF& crsf) {
  const crsf_channels_t* channels_ptr = crsf.getChannelsPacked();
  crsf_channels_t updatedChannels = *channels_ptr;

  // Мапінг значень каналів для шкали від 0 до 100
  int aileronsMapped = map(updatedChannels.ch5, 1000, 2000, 0, 100);
  int elevatorMapped = map(updatedChannels.ch6, 1000, 2000, 0, 100);
  int throttleMapped = map(updatedChannels.ch7, 1000, 2000, 0, 100);
  int rudderMapped = map(updatedChannels.ch8, 1000, 2000, 0, 100);
  int bimbaMapped = map(updatedChannels.ch9, 1000, 2000, 0, 100);

  // Використовуємо ledcWrite для управління 16-бітним PWM сигналом
  SetServoPos(aileronsMapped, aileronsPWMChannel);
  SetServoPos(elevatorMapped, elevatorPWMChannel);
  SetServoPos(throttleMapped, throttlePWMChannel);
  SetServoPos(rudderMapped, rudderPWMChannel);
  SetServoPos(bimbaMapped, bimbaPWMChannel);
}

void sendFallbackChannels() {
  SetServoPos(0, aileronsPWMChannel);
  SetServoPos(0, elevatorPWMChannel);
  SetServoPos(0, throttlePWMChannel);
  SetServoPos(0, rudderPWMChannel);
  SetServoPos(0, bimbaPWMChannel);
}

void loop() {
  crsfIn1.update();
  crsfIn2.update();

  if (crsfIn1.isLinkUp()) {
    if (crsfIn2.isLinkUp()) {
      int lq1 = getLinkQuality(crsfIn1);
      int lq2 = getLinkQuality(crsfIn2);
      if (lq1 >= lq2) {
        sendChannels(crsfIn1);
      } else {
        sendChannels(crsfIn2);
      }
    } else {
      sendChannels(crsfIn1);
    }
  } else {
    if (crsfIn2.isLinkUp()) {
      sendChannels(crsfIn2);
    } else {
      sendFallbackChannels();
    }
  }
}
