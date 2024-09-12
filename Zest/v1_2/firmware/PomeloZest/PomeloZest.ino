#include "esp_sleep.h"

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include <ESP32Time.h>
#include <Preferences.h>

#include <WiFi.h>
#include <ArduinoJson.h>

#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"

#include <Arduino.h>
#include <U8g2lib.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <Adafruit_GPS.h>

#include "qrcode.h"
#include "1euroFilter.h"
#include <EasyButton.h>

// Data file with all web code
#include "uPlot.h"

Preferences preferences;
String wifi_ssid, wifi_pass;
IPAddress wifi_IP;
bool wifiOk, wifiAP;
uint8_t baseMac[6];


U8G2_ST7565_NHD_C12832_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 10, /* data=*/ 0, /* cs=*/ 11, /* dc=*/ 1, /* reset=*/ 22);

AsyncWebServer webServer(80);

Adafruit_GPS GPS(&Serial0);

//WiFiServer server(23);
//WiFiClient client;

#define DISPLAY_MAIN    1
#define DISPLAY_CPM     2
#define DISPLAY_USV     3
#define DISPLAY_WIFI    4
#define DISPLAY_SP      5
#define DISPLAY_POWER   6
#define DISPLAY_LOG_GPS 7
#define DISPLAY_LOG     8
#define DISPLAY_WIFI_QR 9

#define FILTER_LEN      50

// Pin definitions
#define BUT1_A          6
#define BUT2_B          9
#define BUT3_C          7

#define EN_PERI         3
#define MEAS            8
#define HBAT            2
#define BACKLIGHT       15
#define BUZZER          23

#define LOGFILE_INTERVAL_S 60
#define HIST_LEN           60

#define DEBOUNCE_MILLIS 150

const char n42_file[] PROGMEM = "<?xml version=\"1.0\"?><?xml-model href=\"http://physics.nist.gov/N42/2011/schematron/n42.sch\" type=\"application/xml\" schematypens=\"http://purl.oclc.org/dsdl/schematron\"?><RadInstrumentData xmlns=\"http://physics.nist.gov/N42/2011/N42\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://physics.nist.gov/N42/2011/N42 http://physics.nist.gov/N42/2011/n42.xsd\" n42DocUUID=\"d72b7fa7-4a20-43d4-b1b2-7e3b8c6620c1\"><RadInstrumentInformation id=\"RadInstrumentInformation-1\"><RadInstrumentManufacturerName>Pomelo</RadInstrumentManufacturerName><RadInstrumentModelName>Core</RadInstrumentModelName><RadInstrumentClassCode>Radionuclide Identifier</RadInstrumentClassCode><RadInstrumentVersion><RadInstrumentComponentName>Hardware</RadInstrumentComponentName><RadInstrumentComponentVersion>1.2</RadInstrumentComponentVersion></RadInstrumentVersion></RadInstrumentInformation><RadDetectorInformation id=\"RadDetectorInformation-1\"><RadDetectorCategoryCode>Gamma</RadDetectorCategoryCode><RadDetectorKindCode>CsI(Tl)</RadDetectorKindCode></RadDetectorInformation><EnergyCalibration id=\"EnergyCalibration-1\"><CoefficientValues>%C0% %C1% %C2%</CoefficientValues></EnergyCalibration> <RadMeasurement id=\"RadMeasurement-1\"><MeasurementClassCode>Foreground</MeasurementClassCode><StartDateTime>%DATETIME%</StartDateTime><RealTimeDuration>PT%DURATION%S</RealTimeDuration><Spectrum id=\"RadMeasurement-1Spectrum-1\" radDetectorInformationReference=\"RadDetectorInformation-1\" energyCalibrationReference=\"EnergyCalibration-1\"> <LiveTimeDuration>PT%LIVE%S</LiveTimeDuration><ChannelData compressionCode=\"None\">%SPECTRUM%</ChannelData> </Spectrum></RadMeasurement>	</RadInstrumentData>";
const char hist_json[] PROGMEM = "{\"counts\":%COUNTS%,\"time\":%TIME%,\"histo\":[%DATA%]}";
String jsonData;


EasyButton buttonA(BUT1_A, 35, false, false);
EasyButton buttonB(BUT2_B, 35, false, true);
EasyButton buttonC(BUT3_C, 35, false, false);


ESP32Time rtc;

bool sdOk, fileLogging;
File logFile;
uint32_t logFileNumber;
uint32_t logFileEntry;
uint32_t tLog;
char logDir[80];

uint32_t bufHistory[HIST_LEN + 1];

uint32_t tDisplay, tNow, tLastPulse;
char buf[24576];
char printBuf[80];

uint32_t hist[1024];
uint32_t histCounts;
uint32_t histTime;
float histEcal[3];
float histTemp;

volatile bool pressed_BUT1_A, pressed_BUT2_B, pressed_BUT3_C;
volatile bool pressed_BUT1_A_long, pressed_BUT2_B_long, pressed_BUT3_C_long;


RTC_DATA_ATTR bool core_shutdown;
RTC_DATA_ATTR bool spectrumLog;
RTC_DATA_ATTR uint8_t displayMode;



float euroFilterFreq, euroFilterMinCutoff, euroFilterBeta;
OneEuroFilter *euroFilter;

void usbWrite(char* buf)
{
  if (Serial) Serial.write(buf);
}

void IRAM_ATTR isr_BUT1_A()
{
  buttonA.read();
}

void IRAM_ATTR isr_BUT2_B()
{
  buttonB.read();
}

void IRAM_ATTR isr_BUT3_C()
{
  buttonC.read();
}

void onButtonAPressed()
{
  pressed_BUT1_A = true;
}

void onButtonBPressed()
{
  pressed_BUT2_B = true;
}

void onButtonCPressed()
{
  pressed_BUT3_C = true;
}

void onButtonALongPressed()
{
  pressed_BUT1_A_long = true;
}

void onButtonBLongPressed()
{
  pressed_BUT2_B_long = true;
}

void onButtonCLongPressed()
{
  pressed_BUT3_C_long = true;
}



void setup()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  if ((wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) || (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER))
  {
    // Start by making sure the pins are properly accessible
    rtc_gpio_hold_dis(GPIO_NUM_0);
    rtc_gpio_hold_dis(GPIO_NUM_1);
    rtc_gpio_hold_dis(GPIO_NUM_10);
    rtc_gpio_hold_dis(GPIO_NUM_11);
    rtc_gpio_hold_dis(GPIO_NUM_22);
    rtc_gpio_hold_dis(GPIO_NUM_5); // CORE_RX
    rtc_gpio_hold_dis(GPIO_NUM_8); // MEAS
    rtc_gpio_hold_dis(GPIO_NUM_3); // EN_PERI
    rtc_gpio_hold_dis(GPIO_NUM_15); // Backlight
    rtc_gpio_hold_dis(GPIO_NUM_23); // Buzzer
    rtc_gpio_hold_dis(GPIO_NUM_18);
    rtc_gpio_hold_dis(GPIO_NUM_19);
    rtc_gpio_hold_dis(GPIO_NUM_20);
    rtc_gpio_hold_dis(GPIO_NUM_21);
    rtc_gpio_hold_dis(GPIO_NUM_2);   // HBAT
  }

  Serial1.setRxBufferSize(16384);
  //Serial1.begin(115200);  // UART
  Serial1.begin(921600);  // UART
  
  Serial.begin(115200);   // USB CDC

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  preferences.begin("pomeloZest", false);
  wifi_ssid = preferences.getString("ssid", ""); 
  wifi_pass = preferences.getString("password", "");
  preferences.end();
  wifiOk = false;
  wifiAP = false;

  jsonData.reserve(16384);

  u8g2.begin();

  sdOk = false;
  fileLogging = false;


  pinMode(BUT1_A, INPUT);
  pinMode(BUT2_B, INPUT);
  pinMode(BUT3_C, INPUT);

  buttonA.begin();
  buttonB.begin();
  buttonC.begin();

  buttonA.enableInterrupt(isr_BUT1_A);
  buttonB.enableInterrupt(isr_BUT2_B);
  buttonC.enableInterrupt(isr_BUT3_C);

  buttonA.onPressed(onButtonAPressed);
  buttonB.onPressed(onButtonBPressed);
  buttonC.onPressed(onButtonCPressed);

  buttonA.onPressedFor(750, onButtonALongPressed);
  buttonB.onPressedFor(750, onButtonBLongPressed);
  buttonC.onPressedFor(750, onButtonCLongPressed);

  pressed_BUT1_A = false;
  pressed_BUT2_B = false;
  pressed_BUT3_C = false;
  pressed_BUT1_A_long = false;
  pressed_BUT2_B_long = false;
  pressed_BUT3_C_long = false;

  pinMode(EN_PERI, OUTPUT);
  pinMode(MEAS, OUTPUT);
  pinMode(BACKLIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(EN_PERI, LOW);
  digitalWrite(MEAS, HIGH);
  digitalWrite(BACKLIGHT, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(2, LOW);

  tNow = millis();
  tDisplay = tNow;
  tLastPulse = tNow;
  tLog = tNow;

  for (histCounts = 0; histCounts < 1024; histCounts++)
  {
    hist[histCounts] = 0;
  }

  for (histCounts = 0; histCounts < HIST_LEN; histCounts++)
  {
    bufHistory[histCounts] = 0;
  }

  histCounts = 0;
  histTime = 0;
  histTemp = -273;

  euroFilterFreq = 1.0;
  euroFilterMinCutoff = 1.0;
  euroFilterBeta = 0.01;
  euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);

  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED)
  {
    // Hard reset
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,10,"Hard reset");
    u8g2.sendBuffer();
    delay(500);
    spectrumLog = true;

    core_shutdown = false;
    Serial1.write("x\n");

    displayMode = DISPLAY_MAIN;
    Serial1.write("o0\n");
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO)
  {
    // wake up from deep sleep by GPIO
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,10,"GPIO wakeup");
    u8g2.sendBuffer();
    delay(500);
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,10,"EXT1 wakeup");
    sprintf(buf, "0x%X", esp_sleep_get_ext1_wakeup_status());
    u8g2.drawStr(0,20,buf);
    u8g2.sendBuffer();
    delay(500);
    if (core_shutdown)
    {
      core_shutdown = false;
      Serial1.write("x\n");
    }
    Serial1.write("o0\n");
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,10,"Timer wakeup");
    u8g2.sendBuffer();
    delay(500);
  }
}


void zest_lightsleep(uint64_t time)
{
  // Set which pins we need to hold, starting with LCD
  gpio_hold_en(GPIO_NUM_0);
  gpio_hold_en(GPIO_NUM_1);
  gpio_hold_en(GPIO_NUM_10);
  gpio_hold_en(GPIO_NUM_11);
  gpio_hold_en(GPIO_NUM_22);
  gpio_hold_en(GPIO_NUM_5); // CORE_RX
  gpio_hold_en(GPIO_NUM_8); // MEAS
  gpio_hold_en(GPIO_NUM_3); // EN_PERI
  gpio_hold_en(GPIO_NUM_15); // Backlight
  gpio_hold_en(GPIO_NUM_23); // Buzzer
  gpio_hold_en(GPIO_NUM_18);
  gpio_hold_en(GPIO_NUM_19);
  gpio_hold_en(GPIO_NUM_20);
  gpio_hold_en(GPIO_NUM_21);
  gpio_hold_en(GPIO_NUM_2);   // HBAT

  // Light sleep
  gpio_wakeup_enable(GPIO_NUM_6, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_7, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(time*1000);
  //t0 = esp_timer_get_time();
  esp_light_sleep_start();
  //t1 = esp_timer_get_time();

  // Unhold pins
  gpio_hold_dis(GPIO_NUM_0);
  gpio_hold_dis(GPIO_NUM_1);
  gpio_hold_dis(GPIO_NUM_10);
  gpio_hold_dis(GPIO_NUM_11);
  gpio_hold_dis(GPIO_NUM_22);
  gpio_hold_dis(GPIO_NUM_5); // CORE_RX
  gpio_hold_dis(GPIO_NUM_8); // MEAS
  gpio_hold_dis(GPIO_NUM_3); // EN_PERI
  gpio_hold_dis(GPIO_NUM_15); // Backlight
  gpio_hold_dis(GPIO_NUM_23); // Buzzer
  gpio_hold_dis(GPIO_NUM_18);
  gpio_hold_dis(GPIO_NUM_19);
  gpio_hold_dis(GPIO_NUM_20);
  gpio_hold_dis(GPIO_NUM_21);
  gpio_hold_dis(GPIO_NUM_2);   // HBAT

  //return (t1 - t0) / 1000;
}

void zest_deepsleep()
{
  u8g2.setPowerSave(1);
  digitalWrite(EN_PERI, HIGH);

  delay(500);

  rtc_gpio_hold_en(GPIO_NUM_0);
  rtc_gpio_hold_en(GPIO_NUM_1);
  rtc_gpio_hold_en(GPIO_NUM_10);
  rtc_gpio_hold_en(GPIO_NUM_11);
  rtc_gpio_hold_en(GPIO_NUM_22);
  rtc_gpio_hold_en(GPIO_NUM_5); // CORE_RX
  rtc_gpio_hold_en(GPIO_NUM_8); // MEAS
  rtc_gpio_hold_en(GPIO_NUM_3); // EN_PERI
  rtc_gpio_hold_en(GPIO_NUM_15); // Backlight
  rtc_gpio_hold_en(GPIO_NUM_23); // Buzzer
  rtc_gpio_hold_en(GPIO_NUM_18);
  rtc_gpio_hold_en(GPIO_NUM_19);
  rtc_gpio_hold_en(GPIO_NUM_20);
  rtc_gpio_hold_en(GPIO_NUM_21);
  rtc_gpio_hold_en(GPIO_NUM_2);   // HBAT


  // Deep sleep -- now it works, but very bad choice for button placement
  esp_sleep_enable_ext1_wakeup_io(0xC0, ESP_EXT1_WAKEUP_ANY_HIGH);
  rtc_gpio_pulldown_dis(GPIO_NUM_6);
  rtc_gpio_pullup_dis(GPIO_NUM_6);
  rtc_gpio_pulldown_dis(GPIO_NUM_7);
  rtc_gpio_pullup_dis(GPIO_NUM_7);
  esp_deep_sleep_start();
  // Chip dead at this point. Will wake up through reset
}



void drawHistory(uint32_t newValue)
{
  uint32_t historyMax;
  uint16_t i;
  uint8_t s0, s1;

  historyMax = 0;
  bufHistory[HIST_LEN] = newValue; // array is HIST_LEN + 1 sized
  for (i = 0; i < HIST_LEN; i++)
  {
    bufHistory[i] = bufHistory[i + 1];
    if (bufHistory[i] > historyMax) historyMax = bufHistory[i];
  }

  for (i = 0; i < HIST_LEN - 1; i++)
  {
    s0 = 31.0*bufHistory[i]/historyMax;
    s1 = 31.0*bufHistory[i+1]/historyMax;
    u8g2.drawLine(i, 31 - s0, i + 1, 31 - s1);
  }

  // Draw cursor

  i = HIST_LEN;
  s0 = 31.0*bufHistory[HIST_LEN]/historyMax;
  u8g2.drawTriangle(i, 31 - s0, i + 3, 31 - s0 - 3, i + 3, 31 - s0 + 3);

  u8g2.drawLine(64, 0, 64, 32);
}


void drawSpectrum()
{
  uint16_t i;
  char buf[80];
  float spectrum[128];
  float spectrumMax;
  uint8_t s;

  for (i = 0; i < 128; i++)
  {
    spectrum[i] = 0;
  }
  
  for (i = 0; i < 1024; i++)
  {
    spectrum[i/8] += hist[i];
  }

  if (spectrumLog)
  {
    for (i = 0; i < 128; i++)
    {
      if (spectrum[i] >= 1) spectrum[i] = log10(spectrum[i]);
      else spectrum[i] = 0;
    }
  }

  spectrumMax = 0;
  for (i = 0; i < 128; i++)
  {
    if (spectrum[i] > spectrumMax) spectrumMax = spectrum[i];
  }

  u8g2.clearBuffer();
  for (i = 0; i < 128; i++)
  {
    s = 31.0*spectrum[i]/spectrumMax;
    if (s != 0) u8g2.drawLine(i, 31, i, 31 - s);
  }
  u8g2.setFont(u8g2_font_t0_11b_tr);
  if (spectrumLog)
  {
    u8g2.drawStr(0, 10, "L");
    u8g2.drawStr(0, 20, "o");
    u8g2.drawStr(0, 30, "g");
  }
  else
  {
    u8g2.drawStr(0, 10, "L");
    u8g2.drawStr(0, 20, "i");
    u8g2.drawStr(0, 30, "n");
  }
  sprintf(buf, "%d", histCounts);
  u8g2.drawStr(80, 10, buf);
  u8g2.sendBuffer();
  //zest_lightsleep(900);
}

void logSpectrum()
{
  DynamicJsonDocument doc(32768);
  //DeserializationError error = deserializeJson(doc, payload);
  static float lat, lon;
  static uint16_t i;

  // First reset spectrum on Core
  Serial1.write("x");

  // Now save log to currently open file
  if (displayMode == DISPLAY_LOG_GPS)
  {
    lat = GPS.latitudeDegrees;
    lon = GPS.longitudeDegrees;
    if (GPS.lat == 'S') lat = -lat;
    if (GPS.lon == 'W') lon = -lon;

    doc["timestamp"]["year"] = GPS.year;
    doc["timestamp"]["month"] = GPS.month;
    doc["timestamp"]["day"] = GPS.day;
    doc["timestamp"]["hour"] = GPS.hour;
    doc["timestamp"]["minute"] = GPS.minute;
    doc["timestamp"]["seconds"] = GPS.seconds;

    doc["location"]["lat"] = lat;
    doc["location"]["lon"] = lon;
    doc["location"]["speed"] = GPS.speed;
    doc["location"]["fix"] = (int)GPS.fix;
    doc["location"]["fixQ"] = (int)GPS.fixquality;
    doc["location"]["angle"] = GPS.angle;
    doc["location"]["alt"] = GPS.altitude;
    doc["location"]["nSat"] = (int)GPS.satellites;
    doc["location"]["hdop"] = GPS.HDOP;
  }
  else
  {
    doc["timestamp"]["year"] = rtc.getYear();
    doc["timestamp"]["month"] = rtc.getMonth();
    doc["timestamp"]["day"] = rtc.getDay();
    doc["timestamp"]["hour"] = rtc.getHour(true);
    doc["timestamp"]["minute"] = rtc.getMinute();
    doc["timestamp"]["seconds"] = rtc.getSecond();
  }

  doc["spectrum"]["time"] = histTime;
  doc["spectrum"]["counts"] = histCounts;
  doc["spectrum"]["temperature"] = histTemp;
  for (i = 0; i < 1024; i++)
  {
    doc["spectrum"]["hist"][i] = hist[i];
  }

  serializeJson(doc, buf);
  logFile.println(buf);

  logFileEntry++;

  if (logFileEntry > 100)
  {
    logFileEntry = 0;
    logFile.close();

    logFileNumber++;
    sprintf(buf, "/%s/%08lu.csv", logDir, logFileNumber);
    logFile = SD.open(buf, FILE_WRITE);
    if (logFile)
    {
      u8g2.drawStr(0,10,"New file");
    }
    else
    {
      u8g2.drawStr(0,10,"Cannot create new file");
      fileLogging = false;
    }
  }
}

void parseSpectrum(DynamicJsonDocument *doc)
{
  uint16_t i;

  for (i = 0; i < 1024; i++)
  {
    hist[i] = (*doc)["payload"]["data"][i].as<unsigned long long>();
  }

  histCounts = (*doc)["payload"]["count"].as<unsigned long long>();
  histTime = (*doc)["payload"]["time"].as<unsigned long long>();
  histTemp = (*doc)["payload"]["temperature"].as<float>();
  histEcal[0] = (*doc)["payload"]["ecal"][0].as<float>();
  histEcal[1] = (*doc)["payload"]["ecal"][1].as<float>();
  histEcal[2] = (*doc)["payload"]["ecal"][2].as<float>();


  if (displayMode == DISPLAY_SP) drawSpectrum();
  if ((displayMode == DISPLAY_LOG_GPS) && sdOk && fileLogging) logSpectrum();
  if ((displayMode == DISPLAY_LOG) && sdOk && fileLogging) logSpectrum();
}

void parseDosimetry(DynamicJsonDocument *doc)
{
  float displayValue, cpm, uSvph;

  cpm = (*doc)["payload"]["cpm"].as<float>();
  uSvph = (*doc)["payload"]["uSv/h"].as<float>();

  if (displayMode == DISPLAY_CPM)
  {
    displayValue = euroFilter->filter(cpm);

    u8g2.clearBuffer();
    drawHistory((uint32_t)displayValue);
    u8g2.setFont(u8g2_font_t0_11b_tr);
    if (displayValue < 1000)
    {
      u8g2.drawStr(95, 32, "CPM");
      u8g2.setFont(u8g2_font_fub20_tf);
      sprintf(buf, "%3d", (uint16_t)displayValue);
      u8g2.drawStr(83, 22, buf);
    }
    else if (displayValue < 10000)
    {
      u8g2.drawStr(92, 32, "kCPM");
      u8g2.setFont(u8g2_font_fub20_tf);
      sprintf(buf, "%.2f", displayValue/1000.0);
      u8g2.drawStr(73, 22, buf);
    }
    else if (displayValue < 100000)
    {
      u8g2.drawStr(92, 32, "kCPM");
      u8g2.setFont(u8g2_font_fub20_tf);
      sprintf(buf, "%.1f", displayValue/1000.0);
      u8g2.drawStr(73, 22, buf);
    }
    else if (displayValue < 1000000)
    {
      u8g2.drawStr(92, 32, "kCPM");
      u8g2.setFont(u8g2_font_fub20_tf);
      sprintf(buf, "%3d", (uint16_t)(displayValue/1000));
      u8g2.drawStr(83, 22, buf);
    }
    else
    {
      u8g2.drawStr(92, 32, "MCPM");
      u8g2.setFont(u8g2_font_fub20_tf);
      sprintf(buf, "%.2f", displayValue/1000000.0);
      u8g2.drawStr(73, 22, buf);
    }

    u8g2.sendBuffer();
  }
  else if (displayMode == DISPLAY_USV)
  {
    displayValue = euroFilter->filter(uSvph);

    u8g2.clearBuffer();

    drawHistory((uint32_t)(displayValue*1000));

    u8g2.setFont(u8g2_font_t0_11b_tr);
    u8g2.drawStr(89, 32, "uSv/h");
    u8g2.setFont(u8g2_font_fub20_tf);
    if (displayValue < 10)
    {
      sprintf(buf, "%.2f", displayValue);
      u8g2.drawStr(73, 22, buf);
    }
    else
    {
      sprintf(buf, "%3d", (uint16_t)displayValue);
      u8g2.drawStr(83, 22, buf);
    }
    u8g2.sendBuffer();
  }
}

void parseData(char* payload)
{
  DynamicJsonDocument doc(49152);
  DeserializationError error = deserializeJson(doc, payload);

  if (error)
  {
    return;
  }

  if (doc["type"] == "spectrum")
  {
    parseSpectrum(&doc);
  }

  if (doc["type"] == "dosimetry")
  {
    parseDosimetry(&doc);
  }

}

String processorN42(const String& var)
{
  static uint16_t i;
  static char buf[64];
  if(var == "C0")
  {
    sprintf(buf, "%8e", histEcal[0]);
    return String(buf);
  }
  if(var == "C1")
  {
    sprintf(buf, "%8e", histEcal[1]);
    return String(buf);
  }
  if(var == "C2")
  {
    sprintf(buf, "%8e", histEcal[2]);
    return String(buf);
  }
  if(var == "DATETIME")
  {
    // 2003-11-22T23:45:19-07:00
    sprintf(buf, "%s", rtc.getTime("%Y-%m-%dT%H:%M:%S").c_str());
    return String(buf);
  }
  if(var == "DURATION")
  {
    sprintf(buf, "%d", histTime);
    return String(buf);
  }
  if(var == "LIVE")
  {
    sprintf(buf, "%.2f", histTime*1.0 - histCounts*0.00007);  // About 70us dead time per event
    return String(buf);
  }
  if(var == "SPECTRUM")
  {
    jsonData = "";
    for (i = 0; i < 1023; i++)
    {
      sprintf(buf, "%d ", hist[i]);
      jsonData.concat(buf);
    }
    sprintf(buf, "%d", hist[1023]);
    jsonData.concat(buf);
    return jsonData;
  }
  return String();
}

String processorHist(const String& var)
{
  static uint16_t i;
  static char buf[16];
  if(var == "DATA")
  {
    jsonData = "";
    for (i = 0; i < 1023; i++)
    {
      sprintf(buf, "%d,", hist[i]);
      jsonData.concat(buf);
    }
    sprintf(buf, "%d", hist[1023]);
    jsonData.concat(buf);
    return jsonData;
  }
  if(var == "COUNTS")
  {
    sprintf(buf, "%d", histCounts);
    return String(buf);
  }
  if(var == "TIME")
  {
    sprintf(buf, "%d", histTime);
    return String(buf);
  }
  return String();
}

void startServer()
{
  // Static pages, text
  webServer.on("/uPlot.iife.min.js", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/javascript", uPlot_iife_min_js); });
  webServer.on("/uPlot.min.css", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/css", uPlot_min_css); });
  webServer.on("/browserconfig.xml", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", browserconfig_xml); });
  webServer.on("/site.webmanifest", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", site_webmanifest); });

  // Static resources, binary
  webServer.on("/android-chrome-192x192.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", android_chrome_192x192_png, 14447); request->send(response); });
  webServer.on("/android-chrome-512x512.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", android_chrome_512x512_png, 40687); request->send(response); });
  webServer.on("/apple-touch-icon.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", apple_touch_icon_png, 9007); request->send(response); });
  webServer.on("/favicon-16x16.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", favicon_16x16_png, 1106); request->send(response); });
  webServer.on("/favicon-32x32.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", favicon_32x32_png, 2151); request->send(response); });
  webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/vnd.microsoft.icon", favicon_ico, 15086); request->send(response); });
  webServer.on("/mstile-144x144.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_144x144_png, 8925); request->send(response); });
  webServer.on("/mstile-150x150.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_150x150_png, 8717); request->send(response); });
  webServer.on("/mstile-310x150.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_310x150_png, 9366); request->send(response); });
  webServer.on("/mstile-310x310.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_310x310_png, 17934); request->send(response); });
  webServer.on("/mstile-70x70.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_70x70_png, 6320); request->send(response); });
  webServer.on("/safari-pinned-tab.svg", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", safari_pinned_tab_svg); });

  // Dynamic pages start here: index, wifi, time, spectrum
  webServer.on("/spectrum.n42", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "application/octet-stream", n42_file, processorN42); });

  webServer.on("/wifi.htm", HTTP_ANY, [](AsyncWebServerRequest *request)
  {
    int params = request->params();
    bool update;
    update = false;
    for(int i=0;i<params;i++)
    {
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost())
      {
        update = true;
        if (p->name() == "ssid") wifi_ssid = p->value();
        if (p->name() == "pass") wifi_pass = p->value();
      }
    }
    if (update && wifi_ssid != "")
    {
      preferences.begin("pomeloZest", false);
      preferences.putString("ssid", wifi_ssid); 
      preferences.putString("password", wifi_pass);
      preferences.end();
    }
    request->send_P(200, "text/html", wifi_htm); 
  });

  webServer.on("/time.htm", HTTP_ANY, [](AsyncWebServerRequest *request)
  {
    int params = request->params();
    bool update;
    update = false;
    uint16_t yr, mo, da, ho, mi;
    yr = mo = da = ho = mi = -1;
    for(int i=0;i<params;i++)
    {
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost())
      {
        update = true;
        if (p->name() == "year") yr = p->value().toInt();
        if (p->name() == "month") mo = p->value().toInt();
        if (p->name() == "day") da = p->value().toInt();
        if (p->name() == "hour") ho = p->value().toInt();
        if (p->name() == "minute") mi = p->value().toInt();
      }
    }
    if (update && yr != -1 && mo != -1 && da != -1 && ho != -1 && mi != -1)
    {
      rtc.setTime(0, mi, ho, da, mo, yr);
    }
    request->send_P(200, "text/html", time_htm); 
  });

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) 
  {
    if (request->hasParam("restart"))
    {
      if (request->getParam("restart")->value().toInt() == 1) Serial1.write("x");
    }
    if (request->hasParam("stop"))
    {
      if (request->getParam("stop")->value().toInt() == 1) Serial1.write("z");
    }
    request->send_P(200, "text/html", index_htm, processorN42); 
  });
  webServer.on("/hist.json", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "application/json", hist_json, processorHist); });
  webServer.begin();
}

#define XBUTTON_A  16
#define XBUTTON_B  65
#define XBUTTON_C  112

void drawMarkers(bool A, bool B, bool C, bool exitApp)
{
  if (A) u8g2.drawTriangle(XBUTTON_A, 32, XBUTTON_A - 10, 26, XBUTTON_A + 10, 26);
  if (B) u8g2.drawTriangle(XBUTTON_B, 32, XBUTTON_B - 10, 26, XBUTTON_B + 10, 26);
  if (C) u8g2.drawTriangle(XBUTTON_C, 32, XBUTTON_C - 10, 26, XBUTTON_C + 10, 26);
  if (exitApp)
  {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(XBUTTON_A - 12, 10, "Exit");
    u8g2.drawStr(XBUTTON_A - 10, 20, "app");
  }
}

// Needs to be made app-context aware
void showContextHelp()
{
  switch (displayMode)
  {
    case DISPLAY_MAIN:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(XBUTTON_A - 15, 10, "Prev.");
      u8g2.drawStr(XBUTTON_A - 10, 20, "app");

      u8g2.drawStr(XBUTTON_B - 12, 10, "Start");
      u8g2.drawStr(XBUTTON_B - 10, 20, "app");

      u8g2.drawStr(XBUTTON_C - 15, 10, "Next");
      u8g2.drawStr(XBUTTON_C - 10, 20, "app");
      drawMarkers(true, true, true, false);
      u8g2.sendBuffer();
      break;
    case DISPLAY_SP:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);

      u8g2.drawStr(XBUTTON_B - 25, 10, "Log/Lin");
      u8g2.drawStr(XBUTTON_B - 17, 20, "toggle");

      u8g2.drawStr(XBUTTON_C - 20, 10, "Clear");
      u8g2.drawStr(XBUTTON_C - 30, 20, "spectrum");
      drawMarkers(true, true, true, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_CPM:
      u8g2.clearBuffer();
      drawMarkers(true, false, false, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_USV:
      u8g2.clearBuffer();
      drawMarkers(true, false, false, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_WIFI:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(XBUTTON_B - 25, 10, "Connect");
      u8g2.drawStr(XBUTTON_B - 25, 20, "to WiFi");

      u8g2.drawStr(XBUTTON_C - 19, 10, "Start");
      u8g2.drawStr(XBUTTON_C - 25, 20, "hotspot");

      drawMarkers(true, true, true, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_WIFI_QR:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(XBUTTON_B - 15, 10, "Toggle");
      u8g2.drawStr(XBUTTON_B - 17, 20, "QR/text");
      drawMarkers(true, true, false, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_POWER:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(XBUTTON_B - 20, 10, "Power");
      u8g2.drawStr(XBUTTON_B - 8, 20, "off");
      u8g2.drawStr(XBUTTON_C - 13, 20, "Sleep");
      drawMarkers(true, true, true, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_LOG_GPS:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      drawMarkers(true, true, true, true);
      u8g2.sendBuffer();
      break;
    case DISPLAY_LOG:
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(XBUTTON_C - 23, 10, "Mount");
      u8g2.drawStr(XBUTTON_C - 26, 20, "SD card");
      drawMarkers(true, false, true, true);
      u8g2.sendBuffer();
      break;
  }
}


float vbat_read()
{
  float adcVal;

  pinMode(2, INPUT);
  digitalWrite(MEAS, LOW);
  delay(100);
  adcVal = analogRead(2)*0.0049;
  digitalWrite(MEAS, HIGH);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  return adcVal;
}


void loop()
{
  static bool clicks = false;
  static bool backlight = false;
  static uint16_t bufIdx = 0;
  static uint16_t i;
  static uint8_t rxChar;
  static uint8_t menuSelection = DISPLAY_POWER;
  static bool showHelp = false;
  static float vbat;

  buttonA.update();
  buttonB.update();
  buttonC.update();

  // Context independent, button B long press always toggles backlight
  if (pressed_BUT2_B_long)
  {
    pressed_BUT2_B_long = false;
    backlight = !backlight;
    if (backlight) digitalWrite(BACKLIGHT, HIGH);
    else digitalWrite(BACKLIGHT, LOW);
  }

  // Context independent, button C long press always toggles clicks
  if (pressed_BUT3_C_long)
  {
    pressed_BUT3_C_long = false;
    if (clicks) Serial1.write("o0\n");
    else Serial1.write("o1\n");
    clicks = false;
    delay(200);
    while(Serial1.available()) Serial1.read();  // Consume response, hopefully including any spurious 0xFF that would turn clicks back on
  }

  tNow = millis();

  if (pressed_BUT1_A_long || showHelp)
  {
    if (pressed_BUT1_A_long)
    {
      // Initial long press. Display immediately
      tDisplay = tNow;
      // Clear long press flag
      pressed_BUT1_A_long = false;
      // Switch to showHelp flag
      showHelp = true;
    }

    if (tNow >= tDisplay)
    {
      tDisplay += 1000;
      showContextHelp();
    }

    if (buttonA.read() == false)
    {
      // Button released, stop showing help screen.
      showHelp = false;
      
      // Also make sure any buttons fatfingered during
      // help screen don't start messing things up.
      pressed_BUT1_A = false;
      pressed_BUT2_B = false;
      pressed_BUT3_C = false;
      pressed_BUT1_A_long = false;
      pressed_BUT2_B_long = false;
      pressed_BUT3_C_long = false;
      tDisplay = tNow;
    }
  }
  else
  {
    switch (displayMode)
    {
      case DISPLAY_MAIN:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          u8g2.clearBuffer();
          switch (menuSelection)
          {
            case DISPLAY_SP:
              u8g2.setFont(u8g2_font_logisoso16_tf);
              sprintf(buf, "Spectrum");
              i = u8g2.getStrWidth(buf);
              u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, buf);
              u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
              u8g2.drawStr(64 - (16 + 4 + i)/2, 24, "\x58");
              break;
            case DISPLAY_CPM:
              u8g2.setFont(u8g2_font_logisoso16_tf);
              sprintf(buf, "Geiger");
              i = u8g2.getStrWidth(buf);
              u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, buf);
              u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
              u8g2.drawStr(64 - (16 + 4 + i)/2, 24, "\x8D");
              break;
            case DISPLAY_USV:
              u8g2.setFont(u8g2_font_logisoso16_tf);
              sprintf(buf, "uSv/h");
              i = u8g2.getStrWidth(buf);
              u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, buf);
              u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
              u8g2.drawStr(64 - (16 + 4 + i)/2, 24, "\x64");
              break;
            case DISPLAY_WIFI:
              u8g2.setFont(u8g2_font_logisoso16_tf);
              sprintf(buf, "WiFi");
              i = u8g2.getStrWidth(buf);
              u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, buf);
              u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
              u8g2.drawStr(64 - (16 + 4 + i)/2, 24, "\xF7");
              break;
            case DISPLAY_POWER:
              u8g2.setFont(u8g2_font_logisoso16_tf);
              sprintf(buf, "Power");
              i = u8g2.getStrWidth(buf);
              u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, buf);
              u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
              u8g2.drawStr(64 - (16 + 4 + i)/2, 24, "\xEB");
              break;
            case DISPLAY_LOG:
              u8g2.setFont(u8g2_font_logisoso16_tf);
              sprintf(buf, "SD Log");
              i = u8g2.getStrWidth(buf);
              u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, buf);
              u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
              u8g2.drawStr(64 - (16 + 4 + i)/2, 24, "\x80");
              break;
          }
          u8g2.drawTriangle(0, 15, 10, 0, 10, 30);
          u8g2.drawTriangle(127, 15, 117, 0, 117, 30);
          u8g2.sendBuffer();
        }
        break;
      case DISPLAY_SP:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          Serial1.write("h");
        }
        break;
      case DISPLAY_CPM:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          //delay(500);    // perfectly ok
          Serial1.write("m");
        }
        break;
      case DISPLAY_USV:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          //zest_lightsleep(500);   // messes things up. Whaa?
          Serial1.write("m");
        }
        break;
      case DISPLAY_WIFI:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          if (wifiOk)
          {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            if (wifiAP)
            {
              sprintf(buf, "P-%02X%02X, pomelopw", baseMac[4], baseMac[5]);
              u8g2.drawStr(0,10,buf);
            }
            else
            {
              sprintf(buf, "Joined SSID: %s", wifi_ssid);
              u8g2.drawStr(0,10,buf);
            }
            sprintf(buf, "http://%s", wifi_IP.toString().c_str());
            u8g2.drawStr(0,20,buf);
            u8g2.drawStr(57,30,"QR");
            u8g2.sendBuffer();
            Serial1.write("h");
          }
          else
          {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0,10,"WiFi not connected");
            u8g2.drawStr(100,20,"Hot");
            u8g2.drawStr(98,30,"spot");
            if (wifi_ssid != "")
            {
              u8g2.drawLine(95, 11, 95, 32);
              u8g2.drawStr(93 - u8g2.getStrWidth(wifi_ssid.c_str()),20,wifi_ssid.c_str());
              u8g2.drawStr(48,30,"Connect");
            }
            u8g2.sendBuffer();
          }
        }
        break;
      case DISPLAY_WIFI_QR:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          if (wifiOk)
          {
            QRCode qrcode;
            uint8_t qrcodeBytes[qrcode_getBufferSize(3)];

            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);

            if (wifiAP)
            {
              u8g2.drawStr(33,10,"WiFi");
              u8g2.drawStr(33,20,"<---");
              sprintf(buf, "WIFI:S:P-%02X%02X;T:WPA;P:pomelopw;;", baseMac[4], baseMac[5]);
              qrcode_initText(&qrcode, qrcodeBytes, 3, ECC_QUARTILE, buf);
              for (uint8_t y = 0; y < qrcode.size; y++) {
                for (uint8_t x = 0; x < qrcode.size; x++) {
                  if (qrcode_getModule(&qrcode, x, y)) u8g2.drawPixel(x + 1, y + 1);
                }
              }
            }

            u8g2.drawStr(76,10,"  UI");
            u8g2.drawStr(76,20,"--->");
            
            u8g2.drawStr(50,30,"Text");

            sprintf(buf, "HTTP://%s", wifi_IP.toString().c_str());
            qrcode_initText(&qrcode, qrcodeBytes, 3, ECC_HIGH, buf);

            for (uint8_t y = 0; y < qrcode.size; y++) {
              for (uint8_t x = 0; x < qrcode.size; x++) {
                if (qrcode_getModule(&qrcode, x, y)) u8g2.drawPixel(x + 98, y + 1);
              }
            }

            u8g2.sendBuffer();
            Serial1.write("h");
          }
          else
          {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0,10,"WiFi not connected");
            u8g2.drawStr(0,20,"No QR codes to show");
            u8g2.drawStr(40,30,"Text");
            u8g2.sendBuffer();
          }
        }
        break;
      case DISPLAY_POWER:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          vbat = vbat_read();
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          u8g2.drawStr(0, 10, rtc.getTime("%Y.%m.%d").c_str());
          u8g2.drawStr(0, 20, rtc.getTime("%H:%M:%S").c_str());
          sprintf(buf, "Bat: %.2f V", vbat);
          u8g2.drawStr(64, 10, buf);

          u8g2.drawStr(45, 30,"Pw Off");
          u8g2.drawStr(95,30,"Sleep");
          u8g2.sendBuffer();
        }
        break;
      case DISPLAY_LOG_GPS:
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          sprintf(buf, "%02d:%02d %.3f,%.3f", GPS.hour, GPS.minute, GPS.latitudeDegrees, GPS.longitudeDegrees);
          u8g2.drawStr(0,10, buf);
          u8g2.drawStr(0,20,"SD");
          if (sdOk)
          {
            u8g2.drawStr(0,30,"Unmount");
            u8g2.drawStr(60,20,"Log");
            if (fileLogging)
            {
              u8g2.drawStr(57,30,"Stop");
            }
            else
            {
              u8g2.drawStr(57,30,"Start");
            }
          }
          else
          {
            u8g2.drawStr(0,30,"Mount");
          }
          u8g2.sendBuffer();
        }
        break;
      case DISPLAY_LOG:
        if (sdOk && fileLogging)
        {
          if (tNow >= tLog)
          {
            // Trigger a log file write
            tLog += LOGFILE_INTERVAL_S * 1000;
            Serial1.write("h");
          }
        }
        if (tNow >= tDisplay)
        {
          tDisplay += 1000;
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          sprintf(buf, "RTC: %02d:%02d, F: %08lu", rtc.getHour(true), rtc.getMinute(), logFileNumber);
          u8g2.drawStr(0,10, buf);
          sprintf(buf, "L: %03d", logFileEntry);
          u8g2.drawStr(0,20, buf);

          if (sdOk)
          {
            u8g2.drawLine(73, 11, 73, 32);
            u8g2.drawStr(75,20,"Unmount");
            u8g2.drawStr(95,30,"SD");
            u8g2.drawStr(50,20,"Log");
            if (fileLogging)
            {
              u8g2.drawStr(47,30,"Stop");
            }
            else
            {
              u8g2.drawStr(45,30,"Start");
            }
          }
          else
          {
            u8g2.drawStr(90,20,"Mount");
            u8g2.drawStr(100,30,"SD");
          }
          u8g2.sendBuffer();
        }
        break;
    }

    if (pressed_BUT1_A)
    {
      pressed_BUT1_A = false;
      tDisplay = tNow;
      if (displayMode == DISPLAY_MAIN)
      {
        switch (menuSelection)
        {
          case DISPLAY_POWER:
            menuSelection = DISPLAY_CPM;
            break;
          case DISPLAY_CPM:
            menuSelection = DISPLAY_USV;
            break;
          case DISPLAY_USV:
            menuSelection = DISPLAY_SP;
            break;
          case DISPLAY_SP:
            menuSelection = DISPLAY_WIFI;
            break;
          case DISPLAY_WIFI:
            menuSelection = DISPLAY_LOG;
            break;
          case DISPLAY_LOG:
            menuSelection = DISPLAY_POWER;
            break;
        }
      }
      else
      {
        // This is just "Back"/"app exit" for now
        displayMode = DISPLAY_MAIN;

        if (wifiOk)
        {
          webServer.end();
          if (wifiAP)
          {
            WiFi.softAPdisconnect(true); 
          }
          else
          {
            WiFi.disconnect(true, false, 0);
          }
          wifiOk = false;
          wifiAP = false;
        }

        if (sdOk)
        {
            if (fileLogging)
            {
              logFile.close();
              fileLogging = false;
            }
            SD.end();
            sdOk = false;
        }
      }
    }

    if (pressed_BUT2_B)
    {
      pressed_BUT2_B = false;
      if (displayMode == DISPLAY_MAIN)
      {
        // Switch to that application
        displayMode = menuSelection;
        tDisplay = tNow;
        switch (displayMode)
        {
          case DISPLAY_POWER:
            Serial1.write("o0\n");
            break;
          case DISPLAY_CPM:
            Serial1.write("o0\n");
            euroFilterMinCutoff = 0.002670;
            euroFilterBeta = 0.0000153;
            delete euroFilter;
            euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
            for (i = 0; i < HIST_LEN; i++)
            {
              bufHistory[i] = 0;
            }
            break;
          case DISPLAY_USV:
            Serial1.write("o0\n");
            euroFilterMinCutoff = 0.010515;
            euroFilterBeta = 0.042970;
            delete euroFilter;
            euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
            for (i = 0; i < HIST_LEN; i++)
            {
              bufHistory[i] = 0;
            }
            break;
          case DISPLAY_SP:
            Serial1.write("o0\n");
            break;
          case DISPLAY_WIFI:
            Serial1.write("o0\n");
            break;
          case DISPLAY_LOG:
            Serial1.write("o0\n");
            tLog = tNow;
            break;
        }
      }
      else if (displayMode == DISPLAY_SP)
      {
        // log toggle
        spectrumLog = !spectrumLog;
        tDisplay = tNow;
      }
      else if (displayMode == DISPLAY_WIFI)
      {
        if (wifiOk == false)
        {
          if (wifi_ssid != "")
          {
            WiFi.mode(WIFI_STA);
            WiFi.begin(wifi_ssid, wifi_pass);
            for (i = 0; i < 10; i++)
            {
              u8g2.clearBuffer();
              u8g2.setFont(u8g2_font_ncenB08_tr);
              u8g2.drawStr(10,10, "Connection attempt");
              sprintf(buf, "%d/10", i);
              u8g2.drawStr(55,20, buf);
              u8g2.sendBuffer();

              if (WiFi.status() == WL_CONNECTED)
              {
                break;
              }
              delay(300);
            }

            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            if (WiFi.status() == WL_CONNECTED)
            {
              wifi_IP = WiFi.localIP();
              startServer();
              wifiOk = true;
              wifiAP = false;
              u8g2.drawStr(0,10, "Connected to network");
            }
            else
            {
              WiFi.disconnect(true, false, 0);
              u8g2.drawStr(0,10, "Connection failed");
            }
            u8g2.sendBuffer();
            delay(500);
          }
        }
        else
        {
          displayMode = DISPLAY_WIFI_QR;
          Serial1.write("o0\n");
        }
      }
      else if (displayMode == DISPLAY_WIFI_QR)
      {
        displayMode = DISPLAY_WIFI;
        Serial1.write("o0\n");
      }
      else if ((displayMode == DISPLAY_LOG_GPS) || (displayMode == DISPLAY_LOG))
      {
        if (sdOk)
        {
          if (fileLogging)
          {
            // Stop logging
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            logFile.close();
            fileLogging = false;
            u8g2.drawStr(0,10,"STOP LOGGING");
            u8g2.sendBuffer();
            delay(500);
          }
          else
          {
            // Start logging
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            // Find first unused directory
            for (logFileNumber = 0; logFileNumber < 0xFFFFFFFF; logFileNumber++)
            {
              sprintf(logDir, "/%08lu", logFileNumber);
              if (!SD.exists(logDir)) break;
            }
            if (SD.mkdir(logDir))
            {
              logFileNumber = 0;
              logFileEntry = 0;
              sprintf(buf, "/%s/%08lu.csv", logDir, logFileNumber);
              logFile = SD.open(buf, FILE_WRITE);
              if (logFile)
              {
                u8g2.drawStr(0,10,"START LOGGING");
                fileLogging = true;
              }
              else
              {
                u8g2.drawStr(0,10,"Cannot create file");
              }
            }
            else
            {
              u8g2.drawStr(0,10,"Cannot create dir");
            }
            u8g2.sendBuffer();
            delay(500);
          }
        }
      }
      else if (displayMode == DISPLAY_POWER)
      {
        // Power off
        core_shutdown = true;
        Serial1.write("z");
        zest_deepsleep();
      }
    }

    if (pressed_BUT3_C)
    {
      pressed_BUT3_C = false;
      if (displayMode == DISPLAY_MAIN)
      {
        tDisplay = tNow;
        switch (menuSelection)
        {
          case DISPLAY_POWER:
            menuSelection = DISPLAY_LOG;
            break;
          case DISPLAY_CPM:
            menuSelection = DISPLAY_POWER;
            break;
          case DISPLAY_USV:
            menuSelection = DISPLAY_CPM;
            break;
          case DISPLAY_SP:
            menuSelection = DISPLAY_USV;
            break;
          case DISPLAY_WIFI:
            menuSelection = DISPLAY_SP;
            break;
          case DISPLAY_LOG:
            menuSelection = DISPLAY_WIFI;
            break;
        }
      }
      else if (displayMode == DISPLAY_POWER)
      {
        // Sleep
        core_shutdown = false;
        zest_deepsleep();
      }
      else if (displayMode == DISPLAY_SP)
      {
        Serial1.write("x");
      }
      else if (displayMode == DISPLAY_WIFI)
      {
        if (wifiOk == false)
        {
          WiFi.mode(WIFI_AP);
          Network.macAddress(baseMac);
          sprintf(buf, "P-%02X%02X", baseMac[4], baseMac[5]);
          WiFi.softAP(buf, "pomelopw");
          wifi_IP = WiFi.softAPIP();
          startServer();
          wifiOk = true;
          wifiAP = true;
          tDisplay = tNow;
        }
      }
      else if ((displayMode == DISPLAY_LOG_GPS) || (displayMode == DISPLAY_LOG))
      {
        if (sdOk)
        {
            if (fileLogging)
            {
              logFile.close();
              fileLogging = false;
            }
            SD.end();
            sdOk = false;
        }
        else
        {
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          if(SD.begin())
          {
            u8g2.drawStr(0,10,"SD card OK");
            sdOk = true;
            fileLogging = false;
          }
          else
          {
            u8g2.drawStr(0,10,"SD card failed");
            sdOk = false;
            fileLogging = false;
          }
          u8g2.sendBuffer();
          delay(500);
        }
      }
    }
  }

  while (GPS.read() != 0) ;
  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
    if (sdOk && fileLogging)
    {
      // Trigger spectrum readout just on GGA sentence
      if (strncmp(GPS.lastNMEA(), "$GNGGA", 6) == 0) Serial1.write("h");
    }
  }

  if (Serial && (Serial.available()))
  {
    rxChar = Serial.read();
    switch (rxChar)
    {
      case 'd':
        euroFilterMinCutoff += 0.000001;
        sprintf(buf, "fcmin = %.7f, beta = %.7f\n", euroFilterMinCutoff, euroFilterBeta);
        usbWrite(buf);
        delete euroFilter;
        euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
        break;
      case 'c':
        euroFilterMinCutoff -= 0.000001;
        sprintf(buf, "fcmin = %.7f, beta = %.7f\n", euroFilterMinCutoff, euroFilterBeta);
        usbWrite(buf);
        delete euroFilter;
        euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
        break;
      /*
      case 'f':
        euroFilterFreq += 1;
        sprintf(buf, "euroFilterFreq = %.2f\n", euroFilterFreq);
        usbWrite(buf);
        delete euroFilter;
        euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
        break;
      case 'v':
        euroFilterFreq -= 1;
        sprintf(buf, "euroFilterFreq = %.2f\n", euroFilterFreq);
        usbWrite(buf);
        delete euroFilter;
        euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
        break;
      */
      case 'g':
        euroFilterBeta += 0.000001;
        sprintf(buf, "fcmin = %.7f, beta = %.7f\n", euroFilterMinCutoff, euroFilterBeta);
        usbWrite(buf);
        delete euroFilter;
        euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
        break;
      case 'b':
        euroFilterBeta -= 0.000001;
        sprintf(buf, "fcmin = %.7f, beta = %.7f\n", euroFilterMinCutoff, euroFilterBeta);
        usbWrite(buf);
        delete euroFilter;
        euroFilter = new OneEuroFilter(euroFilterFreq, euroFilterMinCutoff, euroFilterBeta);
        break;
    }
  }

  if (Serial1.available())
  {
    while (Serial1.available())
    {
      rxChar = Serial1.read();
      if (rxChar == 0xFF)
      {
        tone(BUZZER, 4000, 2);
        clicks = true;
      }
      else
      {
        if (rxChar == '\n')
        {
          buf[bufIdx] = 0;
          parseData(buf);
          bufIdx = 0;
        }
        else
        {
          buf[bufIdx] = rxChar;
          if (bufIdx < sizeof(buf))
          {
            bufIdx++;
          }
          else
          {
            bufIdx = 0;
          }
        }
      }
    }
  }
}
