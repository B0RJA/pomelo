
typedef struct zestInfo
{
  char name[64];
  String (*getInfo)();
} zestInfo_t;

zestInfo_t zestInfoList[] = 
{
  {"Battery voltage", &app_info_getVbat},
  {"Time", &app_info_getTime},
  {"Free memory", &app_info_getMem},
  {"FW Build Time", &app_info_getBuildTime},
};

int8_t app_info_currentInfo = 0;


bool app_info_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static int8_t numInfos = sizeof(zestInfoList) / sizeof(zestInfo_t);
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    initialized = true;
    app_info_currentInfo = 0;
    serviceRequest(SERVICE_TICK);
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    app_info_currentInfo++;
    if (app_info_currentInfo >= numInfos) app_info_currentInfo = 0;
    retVal = true;
  }
  
  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_TICK);
    runningApp = runningApp->parent;
    retVal = true;
  }

  return retVal;
}

void app_info_draw()
{
  uint8_t strWidth;
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(printBuf, "%s", zestInfoList[app_info_currentInfo].name);
  strWidth = u8g2.getStrWidth(printBuf);
  u8g2.drawStr(64 - (8 + 2 + strWidth)/2 + 8 + 2, 10, printBuf);
  sprintf(printBuf, "%s", zestInfoList[app_info_currentInfo].getInfo().c_str());
  strWidth = u8g2.getStrWidth(printBuf);
  u8g2.drawStr(64 - (8 + 2 + strWidth)/2 + 8 + 2, 20, printBuf);
  u8g2.drawStr(95,30,"Next");
}


String app_info_getMem()
{
  return String(ESP.getFreeHeap());
}


String app_info_getTime()
{
  return rtc.getTime("%Y.%m.%d %H:%M:%S");
}

String app_info_getBuildTime()
{
  sprintf(printBuf, "%s %s", __DATE__, __TIME__);
  return String(printBuf);
}

String app_info_getVbat()
{
  sprintf(printBuf, "%.2f V", vbat_read());
  return String(printBuf);
}


/*
void app_power_draw()
{
  float vbat;

  vbat = vbat_read();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, rtc.getTime("%Y.%m.%d").c_str());
  u8g2.drawStr(0, 20, rtc.getTime("%H:%M:%S").c_str());
  sprintf(printBuf, "Bat: %.2f V", vbat);
  u8g2.drawStr(64, 10, printBuf);

  u8g2.drawStr(45, 30,"Pw Off");
  u8g2.drawStr(95,30,"Sleep");
}
*/


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


