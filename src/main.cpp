#define BLYNK_TEMPLATE_ID "TMPL6bsROhWmq"
#define BLYNK_TEMPLATE_NAME "DucHome"
#define BLYNK_AUTH_TOKEN "Y55whuX8Zhs332sjJ9pBK_QSS0LoiRPk"

#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <stdint.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"
#include "Audio.h"
#include "scheduler.h"

#define servo_pin 13
#define light_pin 16
#define led_pin 12
#define DHTTYPE    DHT11     // DHT 11
#define DHTPIN 14
#define PIR_PIN 27
#define MAX98357A_I2S_DOUT  15
#define MAX98357A_I2S_BCLK 2
#define MAX98357A_I2S_LRC  4
//

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27,16,02);
Adafruit_NeoPixel strip(4, led_pin, NEO_GRB + NEO_KHZ800);
Servo door;
ListTask Ltask;
WiFiClient client;
Audio audio;
hw_timer_s * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
const char * ssid = "ahaha";
const char * pass = "123456789";
bool door_state = 0;
bool light_sensor =  0;
bool led_state = 0;
bool motion_mode = 0;
bool light_mode = 0;
volatile bool motionDetected = false;


void led_on();
void led_off();
void door_open();
void door_close();
void debug();
void read_light_sensor();
void IRAM_ATTR onTimer();
void read_DHT11();
void IS_led_auto_mode();
void IRAM_ATTR detectMotion();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // DHT 11
  dht.begin();
  //
  pinMode(light_pin, INPUT);
  pinMode(PIR_PIN, INPUT);
  //
  timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true);
  timerAlarmEnable(timer);
  //
  lcd.init();
  lcd.backlight();
  lcd.clear();
  //
  WiFi.begin(ssid, pass);
  Serial.print("Connecting WiFi...");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  //
  door.attach(servo_pin);
  //
  strip.begin();
  strip.show();
  //
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  //
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), detectMotion, RISING);
  //
  audio.setPinout(MAX98357A_I2S_BCLK, MAX98357A_I2S_LRC, MAX98357A_I2S_DOUT);
  audio.setVolume(15);
  audio.connecttohost("http://vis.media-ice.musicradio.com/HeartNorthWestMP3");
  //
  Ltask.SCH_Add_Task(debug, 1000, 1000);
  Ltask.SCH_Add_Task(read_light_sensor, 2000, 3000);
  Ltask.SCH_Add_Task(read_DHT11, 1000, 5000);
  Ltask.SCH_Add_Task(IS_led_auto_mode, 3000, 1000);
}
BLYNK_WRITE(V0){
  bool button = param.asInt();
  if(button){
      led_on();
  }else{
      led_off();
  }
}
BLYNK_WRITE(V1){
  bool button = param.asInt();
  if(button){
    door_open();
  }else{
    door_close();
  }
}
BLYNK_WRITE(V4){
  motion_mode = param.asInt();
}
BLYNK_WRITE(V5){
  light_mode = param.asInt();
}
void loop() {
  Ltask.SCH_Dispatch_Task();
  Blynk.run();
  audio.loop();
  // Serial.print("TEST PIR: ");
  // Serial.println(digitalRead(PIR_PIN));
  // delay(100);
}

void door_open(){
  door_state = 1;
  door.write(80);
} 
void door_close(){
  door_state = 0;
  door.write(0);
}
void read_light_sensor(){
  light_sensor = digitalRead(light_pin);
}
void debug(){
  Serial.println("TEST");
}
void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  Ltask.run();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void led_on(){
  led_state = 1;
  strip.setPixelColor(0, strip.Color(255, 255, 255));
  strip.setPixelColor(1, strip.Color(255, 255, 255)); 
  strip.setPixelColor(2, strip.Color(255, 255, 255));  
  strip.setPixelColor(3, strip.Color(255, 255, 255));
  strip.show();
}
void led_off(){
  led_state = 0;
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.setPixelColor(1, strip.Color(0, 0, 0)); 
  strip.setPixelColor(2, strip.Color(0, 0, 0));  
  strip.setPixelColor(3, strip.Color(0, 0, 0));
  strip.show();
}
void read_DHT11(){
  uint8_t h= dht.readHumidity();
  uint8_t t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V3, h);
  // Serial.print(F("Humidity: "));
  // Serial.println(h);
  // Serial.print(F("%  Temperature: "));
  // Serial.println(t);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("NHIET DO ");
  lcd.print(t);
  lcd.print("*C");
  lcd.setCursor(0, 1);
  lcd.print("DO AM ");
  lcd.print(h);
  lcd.print("%");
}
void IS_led_auto_mode(){
  if(motion_mode){
    // Serial.println("IN MODE AUTO LED");
    if(motionDetected){
      // Serial.println("DETECTED!!!");
      if(!led_state){
        // Serial.println("ON LED");
        led_on();
        Ltask.SCH_Add_Task(led_off, 3000, 0);
      }
      motionDetected = 0;
    }
  }
  if(light_mode){
    if(digitalRead(light_pin)){
      led_on();
    } else {
      led_off();
    }
  }
}
void IRAM_ATTR detectMotion() {
  motionDetected = true; 
}