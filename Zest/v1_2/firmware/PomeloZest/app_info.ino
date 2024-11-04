
typedef struct zestInfo
{
  char name[64];
  String (*getInfo)();
} zestInfo_t;

zestInfo_t zestInfoList[] = 
{
  {"Free memory", &app_info_getMem},
  {"Time", &app_info_getTime},
  {"FW Build Time", &app_info_getBuildTime},
};

int8_t app_info_currentInfo = 0;


bool app_info_updateUI(uint8_t buttonFlags)
{
  static int8_t numInfos = sizeof(zestInfoList) / sizeof(zestInfo_t);
  static bool initialized = false;

  if (!initialized)
  {
    initialized = true;
    app_info_currentInfo = 0;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    app_info_currentInfo++;
    if (app_info_currentInfo >= numInfos) app_info_currentInfo = 0;
  }

  app_info_render();
  
  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

void app_info_render()
{
  uint8_t strWidth;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(printBuf, "%s", zestInfoList[app_info_currentInfo].name);
  strWidth = u8g2.getStrWidth(printBuf);
  u8g2.drawStr(64 - (8 + 2 + strWidth)/2 + 8 + 2, 10, printBuf);
  sprintf(printBuf, "%s", zestInfoList[app_info_currentInfo].getInfo().c_str());
  strWidth = u8g2.getStrWidth(printBuf);
  u8g2.drawStr(64 - (8 + 2 + strWidth)/2 + 8 + 2, 20, printBuf);
  u8g2.drawStr(95,30,"Next");
  u8g2.sendBuffer();
}

bool app_info_updateService(uint8_t serviceFlags)
{
  return true;
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
