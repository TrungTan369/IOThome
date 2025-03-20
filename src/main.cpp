
#include <Arduino.h>
#include <WiFi.h>
#include "Audio.h"

#define MAX98357A_I2S_DOUT  25
#define MAX98357A_I2S_BCLK 27
#define MAX98357A_I2S_LRC  26
//

WiFiClient client;
Audio audio;

const char * ssid = "ahaha";
const char * pass = "123456789";



void setup() {
  Serial.begin(115200);
  // DHT 11
  //
  WiFi.begin(ssid, pass);
  Serial.print("Connecting WiFi...");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  audio.setPinout(MAX98357A_I2S_BCLK, MAX98357A_I2S_LRC, MAX98357A_I2S_DOUT);
  audio.setVolume(21);
  audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");
  //

}

void loop() {
  audio.loop();
}

