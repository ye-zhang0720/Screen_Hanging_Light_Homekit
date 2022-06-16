/*
   light_controller_led.ino

   GPIO: W 16
         c 14

    Created on: 2022-04-19
        Author: Ye Zhang
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <arduino_homekit_server.h>
#include "ButtonDebounce.h"
#include "ButtonHandler.h"
#include <Ticker.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFi_config.h"
#include <ArduinoJson.h>
#include <DFRobot_PAJ7620U2.h>


#define Warm_GPIO 14
#define Cold_GPIO 13

#define APK_MAX_COLORTEMP 400 //用户定义最大的色温数值
#define APK_MID_COLORTEMP 225 //用户定义中间的色温数值
#define APK_MIN_COLORTEMP 50  //用户定义最小的色温数值

#define SIMPLE_INFO(fmt, ...) printf_P(PSTR(fmt "\n"), ##__VA_ARGS__);


//#define __DEBUG 1

#define key_pin 0

Ticker ticker;
WiFi_config wifiManager;
DFRobot_PAJ7620U2 paj;

ButtonDebounce btn(key_pin, INPUT_PULLUP, LOW);
ButtonHandler btnHandler;

//设备信息
#define ChineseWordNumber 4
#define EnglishWordNumber 0
char name_generation[EnglishWordNumber + ChineseWordNumber * 3 + 13] = "屏幕挂灯";
char categoryNo = homekit_accessory_category_bridge;
uint8_t MAC_array_STA[6];
char SN[13];
char Setup_ID[5];
char Setup_code[11];

int outWarmChannle = 0;
int outColdChannle = 0;

bool configure_network_mode = false; //配网模式标志

extern "C" float led_color_tempearture;
extern "C" float led_brightness;
extern "C" bool led_power;

float current_led_color_tempearture = 0;
float current_led_brightness = 0;

void to_Led_Pwm(float _color_temperature, float _brightness)
{
  float temp_color_temperature = (_color_temperature - APK_MIN_COLORTEMP) / (APK_MAX_COLORTEMP - APK_MIN_COLORTEMP);
  _brightness = _brightness / 100;
  float cold_tmp = temp_color_temperature * _brightness;
  float warm_tmp = (1 - temp_color_temperature) * _brightness;

  outWarmChannle = (int)510 * (1 - warm_tmp);
  outColdChannle = (int)510 * (1 - cold_tmp);
}

void led_update()
{
  if ((led_brightness == 0) && (current_led_brightness == 0)) //当前亮度和目标亮度均等于零
  {
    return;
  }

  if ((led_color_tempearture == current_led_color_tempearture) && (led_brightness == current_led_brightness)) //目标亮度和当前亮度相等 目标色温和当前色温相等
  {
    return;
  }

  if (led_color_tempearture > current_led_color_tempearture)
  {
    float temp = current_led_color_tempearture + 3;
    if (temp > led_color_tempearture)
    {
      current_led_color_tempearture = led_color_tempearture;
    }
    else
    {
      current_led_color_tempearture = temp;
    }
  }
  else if (led_color_tempearture < current_led_color_tempearture)
  {
    float temp = current_led_color_tempearture - 3;
    if (temp < led_color_tempearture)
    {
      current_led_color_tempearture = led_color_tempearture;
    }
    else
    {
      current_led_color_tempearture = temp;
    }
  }

  if (led_brightness > current_led_brightness)
  {
    current_led_brightness = current_led_brightness + 1;
  }
  else if (led_brightness < current_led_brightness)
  {
    current_led_brightness = current_led_brightness - 1;
  }

  to_Led_Pwm(current_led_color_tempearture, current_led_brightness);
#ifdef __DEBUG
  // Serial.print("led_brightness:");
  // Serial.println(led_brightness);
  // Serial.print("current_led_brightness:");
  // Serial.println(current_led_brightness);
  // Serial.print("led_color_tempearture:");
  // Serial.println(led_color_tempearture);
  // Serial.print("current_led_color_tempearture:");
  // Serial.println(current_led_color_tempearture);
  // Serial.print("warm:");
  // Serial.println(outWarmChannle);
  // Serial.print("Cold:");
  // Serial.println(outColdChannle);
  // Serial.println();
#endif
  analogWrite(Warm_GPIO, outWarmChannle);
  analogWrite(Cold_GPIO, outColdChannle);

  return;
}

void config_network_function()
{
  if (configure_network_mode == true)
  {
    if (current_led_brightness == 80)
    {
      led_brightness = 0;
      led_color_tempearture = 255;
    }
    else if (current_led_brightness == 0)
    {
      led_brightness = 50;
      led_color_tempearture = 255;
    }
    // Serial.println(current_led_brightness);
  }
}

void tick()
{
  config_network_function();
  if ((led_power == false) && (current_led_brightness != 0) && (configure_network_mode == false))
  {
    led_brightness = 0;
  }
  led_update();
}

// gets called when WiFiManager enters configuration mode
void configModeCallback(WiFi_config *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  // entered config mode, make led toggle faster
}


void IRAM_ATTR btnInterrupt() {
  btn.update();
}


void setup()
{
  Serial.begin(115200);

  analogWriteRange(512);
  analogWriteFreq(2000);

  pinMode(Warm_GPIO, OUTPUT);
  pinMode(Cold_GPIO, OUTPUT);
  digitalWrite(Warm_GPIO, LOW);
  digitalWrite(Cold_GPIO, LOW);

  configure_network_mode = true;
  ticker.attach(0.02, tick);

  Serial.println("Gesture recognition system base on PAJ7620U2");
  while (paj.begin() != 0)
  {
    Serial.println("initial PAJ7620U2 failure! Please check if all the connections are fine, or if the wire sequence is correct?");
    delay(500);
  }
  Serial.println("PAJ7620U2 init completed, start to test the gesture recognition function");

  paj.setGestureHighRate(true);



  btn.setCallback(std::bind(&ButtonHandler::handleChange, &btnHandler,
                            std::placeholders::_1));
  btn.setInterrupt(btnInterrupt);
  btnHandler.setIsDownFunction(std::bind(&ButtonDebounce::checkIsDown, &btn));
  btnHandler.setCallback([](button_event e)
  {
    if (e == BUTTON_EVENT_DOUBLECLICK) {
      configure_network_mode = true;
      wifiManager.resetSettings();
      delay(2000);
      ESP.reset();
    }
    else if (e == BUTTON_EVENT_LONGCLICK)
    {
      SIMPLE_INFO("Button Event: LONGCLICK");
      SIMPLE_INFO("Rebooting...");
      configure_network_mode = true;
      homekit_storage_reset();
      wifiManager.resetSettings();
      ESP.restart(); // or system_restart();
    }
  });

  WiFi.macAddress(MAC_array_STA);
  GenerateSerialNumber(SN);
  for (int i = EnglishWordNumber + ChineseWordNumber * 3; i < EnglishWordNumber + ChineseWordNumber * 3 + 13; i++)
  {
    if (i - (EnglishWordNumber + ChineseWordNumber * 3) < 13)
    {
      name_generation[i] = SN[i - (EnglishWordNumber + ChineseWordNumber * 3)];
    }
  }
  GenerateSetupPassword(Setup_ID, Setup_code);
  Serial.println(name_generation);
  Serial.print("Setup_code:");
  Serial.print(Setup_code);
  Serial.print("  Setup_ID:");
  Serial.println(Setup_ID);



  wifiManager.setConnectTimeout(300);
  wifiManager.setDebugOutput(false);


  //设置配对信息
  wifiManager.setHomekitRepairParameter(SN, Setup_ID, Setup_code);

  // reset settings - for testing
  // wifiManager.resetSettings();
  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Homekit_Light"))
  {
    Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  // if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  homekit_setup();

  configure_network_mode = false;
  led_brightness = 0;
  ticker.attach(0.01, tick);
}

void loop()
{
  homekit_loop();
}

//==============================
// Homekit setup and loop
//==============================

extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t name;
extern "C" homekit_characteristic_t serial_number;
extern "C" homekit_characteristic_t cha_led_on;
extern "C" homekit_characteristic_t cha_led_color_temperature;
extern "C" homekit_characteristic_t cha_led_brightness;

extern "C" homekit_characteristic_t cha_top2bottom_motion;
extern "C" homekit_characteristic_t cha_bottom2top_motion;
extern "C" homekit_characteristic_t cha_left2right_motion;
extern "C" homekit_characteristic_t cha_right2left_motion;

void GenerateSerialNumber(char *_SN)
{
  /*
    序列号定义
    SN_   _ _           _               _         _            _          _        _       _
        分类编号 (mac[0]+mac[5])%10  mac[1]对应字母 mac[2]%10 mac[3]%10  mac[4]%10  随机数  随机字母
  */
  char words[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"; // 36个字符

  _SN[0] = 'S';
  _SN[1] = 'N';
  _SN[2] = '_';
  _SN[3] = (char)(categoryNo / 10) % 10 + 48;
  _SN[4] = categoryNo % 10 + 48;
  _SN[5] = (MAC_array_STA[0] + MAC_array_STA[5]) % 10 + 48;
  _SN[6] = words[MAC_array_STA[1] % 36];
  _SN[7] = MAC_array_STA[2] % 10 + 48;
  _SN[8] = words[(MAC_array_STA[3] * MAC_array_STA[2] + 6) % 36];
  _SN[9] = MAC_array_STA[4] % 10 + 48;
  _SN[10] = (char)((MAC_array_STA[3] * MAC_array_STA[2] + MAC_array_STA[2] * 0.86425) * 0.5) % 10 + 48;
  _SN[11] = words[((char)((MAC_array_STA[5] * 6 / MAC_array_STA[2] + MAC_array_STA[4] * 0.86425) * 0.5 + 0.5)) % 36];
  _SN[12] = '\0';
}

void GenerateSetupPassword(char *_setupID, char *_setupcode)
{
  char words[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"; // 36个字符

  _setupID[0] = words[((char)((MAC_array_STA[5] * 4.3 / MAC_array_STA[3] + MAC_array_STA[0] * 0.86425) * 0.5 + 0.5)) % 36];
  _setupID[1] = words[((char)((MAC_array_STA[1] * 2 / (MAC_array_STA[2] * 0.3) + MAC_array_STA[4] * 3) * 0.9 + 0.5)) % 36];
  _setupID[2] = words[((char)((MAC_array_STA[3] * MAC_array_STA[2] + 425) * 0.5 + 0.5)) % 36];
  _setupID[3] = words[((char)((MAC_array_STA[4] / MAC_array_STA[1] + MAC_array_STA[5] * 0.86425) * 0.5 + 25.5)) % 36];
  _setupID[4] = '\0';
  /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
  _setupcode[0] = (MAC_array_STA[0] + MAC_array_STA[5]) % 10 + 48;
  _setupcode[1] = MAC_array_STA[2] % 10 + 48;
  _setupcode[2] = ((char)((MAC_array_STA[3] * MAC_array_STA[2] + MAC_array_STA[2] * 0.86425) * 0.5) % 10) + 48;
  _setupcode[3] = 0x2d;
  _setupcode[4] = (categoryNo * MAC_array_STA[0] + MAC_array_STA[5]) % 10 + 48;
  _setupcode[5] = (categoryNo * 16 + MAC_array_STA[5] * MAC_array_STA[1]) % 10 + 48;
  _setupcode[6] = 0x2d;
  _setupcode[7] = (MAC_array_STA[0] + MAC_array_STA[1] + MAC_array_STA[2] + MAC_array_STA[3] + MAC_array_STA[4] + MAC_array_STA[5]) % 10 + 48;
  _setupcode[8] = ((MAC_array_STA[0] + MAC_array_STA[1] + MAC_array_STA[2]) * (MAC_array_STA[3] + MAC_array_STA[4] + MAC_array_STA[5])) % 10 + 48;
  _setupcode[9] = ((MAC_array_STA[0] + MAC_array_STA[5] + MAC_array_STA[3]) * (MAC_array_STA[3] + MAC_array_STA[2] + MAC_array_STA[5])) % 10 + 48;
  _setupcode[10] = '\0';
}

void homekit_setup()
{

  name.value = HOMEKIT_STRING_CPP(name_generation);
  serial_number.value = HOMEKIT_STRING_CPP(SN);
  config.password = Setup_code;
  config.setupId = Setup_ID;

  arduino_homekit_setup(&config);
}

bool gesture_flag = false;
#define show_gesture_time 1000

void homekit_loop()
{
  btnHandler.loop();
  arduino_homekit_loop();
  static uint32_t next_heap_millis = 0;
  static uint32_t next_check_gesture_millis = 0;
  uint32_t time = millis();
  if (time > next_heap_millis)
  {
    SIMPLE_INFO("heap: %d, sockets: %d",
                ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
    next_heap_millis = time + 5000;
  }

  if (gesture_flag)
  {
    if (time > next_check_gesture_millis)
    {
      cha_top2bottom_motion.value.bool_value = false;
      homekit_characteristic_notify(&cha_top2bottom_motion, cha_top2bottom_motion.value);

      cha_bottom2top_motion.value.bool_value = false;
      homekit_characteristic_notify(&cha_bottom2top_motion, cha_bottom2top_motion.value);

      cha_left2right_motion.value.bool_value = false;
      homekit_characteristic_notify(&cha_left2right_motion, cha_left2right_motion.value);

      cha_right2left_motion.value.bool_value = false;
      homekit_characteristic_notify(&cha_right2left_motion, cha_right2left_motion.value);

      gesture_flag = false;
    }

  }


  DFRobot_PAJ7620U2::eGesture_t gesture = paj.getGesture();
  if (gesture != paj.eGestureNone)
  {
    /* Get the string descritpion corresponding to the gesture number.
       The string description could be
       "None","Right","Left", "Up", "Down", "Forward", "Backward", "Clockwise", "Anti-Clockwise", "Wave",
       "WaveSlowlyDisorder", "WaveSlowlyLeftRight", "WaveSlowlyUpDown", "WaveSlowlyForwardBackward"
    */
    String description = paj.gestureDescription(gesture); // Convert gesture number into string description
    if (gesture == paj.eGestureRight)                     // 上-》下
    {

      cha_top2bottom_motion.value.bool_value = true;
      homekit_characteristic_notify(&cha_top2bottom_motion, cha_top2bottom_motion.value);
      gesture_flag = true;
      next_check_gesture_millis = time + show_gesture_time;
    }
    else if (gesture == paj.eGestureLeft) //下->上
    {

      cha_bottom2top_motion.value.bool_value = true;
      homekit_characteristic_notify(&cha_bottom2top_motion, cha_bottom2top_motion.value);
      gesture_flag = true;
      next_check_gesture_millis = time + show_gesture_time;
    }
    else if (gesture == paj.eGestureUp) //左-》右
    {

      cha_left2right_motion.value.bool_value = true;
      homekit_characteristic_notify(&cha_left2right_motion, cha_left2right_motion.value);
      gesture_flag = true;
      next_check_gesture_millis = time + show_gesture_time;
    }
    else if (gesture == paj.eGestureDown) //右-》左
    {
      cha_right2left_motion.value.bool_value = true;
      homekit_characteristic_notify(&cha_right2left_motion, cha_right2left_motion.value);
      gesture_flag = true;
      next_check_gesture_millis = time + show_gesture_time;
    }
    // Serial.println("--------------Gesture Recognition System---------------------------");
    // Serial.print("gesture code        = ");
    // Serial.println(gesture);
    // Serial.print("gesture description  = ");
    // Serial.println(description);
    // Serial.println();
  }
}
