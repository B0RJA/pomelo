
typedef struct app_settings_zestSetting
{
  char name[64];
  String (*currentVal)();
  void (*changeVal)();
} app_settings_zestSetting_t;

app_settings_zestSetting_t app_settings_zestSettingsList[] = 
{
  {"Clicks", &app_settings_clicks_currentVal, &app_settings_clicks_changeVal},
  {"Backlight", &app_settings_backlight_currentVal, &app_settings_backlight_changeVal},
  {"Backlight intensity", &app_settings_backlightIntensity_currentVal, &app_settings_backlightIntensity_changeVal},
  {"Backlight duration", &app_settings_backlightDuration_currentVal, &app_settings_backlightDuration_changeVal},
  {"Click duration", &app_settings_clickDuration_currentVal, &app_settings_clickDuration_changeVal},
};

int8_t app_settings_currentSetting = 0;
int8_t app_settings_numSettings = sizeof(app_settings_zestSettingsList) / sizeof(app_settings_zestSetting_t);

bool app_settings_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;

  if (!initialized)
  {
    initialized = true;
    serviceRequest(SERVICE_CONFIG);
    app_settings_currentSetting = 0;
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    app_settings_zestSettingsList[app_settings_currentSetting].changeVal();
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    app_settings_currentSetting++;
    if (app_settings_currentSetting >= app_settings_numSettings) app_settings_currentSetting = 0;
  }

  app_settings_display();
  
  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_CONFIG);
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

bool app_settings_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_CONFIG)) != 0)
  {
    app_settings_display();
  }
  return true;
}

void app_settings_display()
{
  uint8_t strWidth;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(printBuf, "%s", app_settings_zestSettingsList[app_settings_currentSetting].name);
  strWidth = u8g2.getStrWidth(printBuf);
  u8g2.drawStr(64 - (8 + 2 + strWidth)/2 + 8 + 2, 10, printBuf);
  sprintf(printBuf, "%s", app_settings_zestSettingsList[app_settings_currentSetting].currentVal().c_str());
  strWidth = u8g2.getStrWidth(printBuf);
  u8g2.drawStr(64 - (8 + 2 + strWidth)/2 + 8 + 2, 20, printBuf);
  u8g2.drawStr(45, 30,"Change");
  u8g2.drawStr(95,30,"Next");
  u8g2.sendBuffer();
}

String app_settings_clicks_currentVal()
{
  if (serviceUartPulse) return "ON";
  else return "OFF";
}

void app_settings_clicks_changeVal()
{
  if (!serviceUartPulse)
  {
    coreCommand("o2\n");
    // Hack to make screen update quickly.
    // But this will be overwritten with the
    // "true" value on the next service call
    serviceUartPulse = true;
  }
  else
  {
    coreCommand("o0\n");
    serviceUartPulse = false;
  }
}

String app_settings_backlight_currentVal()
{
  if (backlightEnabled) return "ON";
  else return "OFF";
}

void app_settings_backlight_changeVal()
{
  backlightEnabled = !backlightEnabled;
  if (backlightEnabled) backlight(3);
  else backlight(0);
}

String app_settings_backlightIntensity_currentVal()
{
  return String(backlightIntensity);
}

void app_settings_backlightIntensity_changeVal()
{
  if (backlightIntensity == 255) backlightIntensity = 150;
  else if (backlightIntensity == 150) backlightIntensity = 50;
  else if (backlightIntensity == 50) backlightIntensity = 10;
  else if (backlightIntensity == 10) backlightIntensity = 1;
  else backlightIntensity = 255;
  backlight(3); // Switch to level immediately
}

String app_settings_backlightDuration_currentVal()
{
  if (backlightSeconds == 0) return "Infinite";
  else return String(backlightSeconds) + " s";
}

void app_settings_backlightDuration_changeVal()
{
  if (backlightSeconds == 0) backlightSeconds = 5;
  else if (backlightSeconds == 5) backlightSeconds = 10;
  else if (backlightSeconds == 10) backlightSeconds = 60;
  else backlightSeconds = 0;
}

String app_settings_clickDuration_currentVal()
{
  if (beep_ms == 2) return "Short";
  else return "Long";
}

void app_settings_clickDuration_changeVal()
{
  if (beep_ms == 2) beep_ms = 20;
  else beep_ms = 2;
}

