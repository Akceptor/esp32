#include <WiFi.h>
#include <WiFiClientSecure.h>
//https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/archive/master.zip
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
//https://codeload.github.com/taranais/NTPClient/zip/refs/heads/master
#include <NTPClient.h>

const char* ssid = "MyWifi";
const char* password = "12345678";

#define BOTtoken "XXX:YYY"
#define CHAT_ID "-12345"
#define PWR_GPIO 5

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String formattedDate;
String timeStamp;
int State;
int PrevState;

void setup() {
  pinMode(PWR_GPIO, INPUT);
  Serial.begin(115200);  // Initialize serial communication

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  timeClient.begin();
  timeClient.setTimeOffset(7200);  //GMT+2

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println(WiFi.localIP());
  delay(5000);
  client.setInsecure();  // Bypass SSL for testing purposes
  bot.sendMessage(CHAT_ID, "😎 Бот готовий: " + WiFi.localIP().toString(), "");
}

void loop() {
  State = digitalRead(PWR_GPIO);
  if (PrevState != State) {
    //Get time
    while (!timeClient.update()) {
      timeClient.forceUpdate();
    }
    formattedDate = timeClient.getFormattedDate();
    int splitT = formattedDate.indexOf("T");
    timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
    //Message
    if (State == HIGH) {
      bot.sendMessage(CHAT_ID, "✅ Світло з`явилося о " + timeStamp +" 👌", "");
    } else {
      bot.sendMessage(CHAT_ID, "❌ Світло зникло о " + timeStamp +" 😱", "");
    }
    PrevState = State;
  }
  delay(5000);
}
