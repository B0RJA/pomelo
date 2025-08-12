
bool app_clock_showDose;
bool app_clock_setTime;
int16_t app_clock_setTime_hour, app_clock_setTime_minute;

bool app_clock_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    serviceRequest(SERVICE_DOSIMETRY);
    initialized = true;
    app_clock_showDose = false;
    app_clock_setTime = false;
    retVal = true;
  }

  if ((serviceFlags & (1 << SERVICE_DOSIMETRY)) != 0)
  {
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_3_LONG)) != 0)
  {
    app_clock_setTime = true;
    app_clock_setTime_hour = rtc.getHour(true);
    app_clock_setTime_minute = rtc.getMinute();
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    if (app_clock_setTime)
    {
      // Set current time and exit this state
      rtc.setTime(0, app_clock_setTime_minute, app_clock_setTime_hour, rtc.getDay(), rtc.getMonth() + 1, rtc.getYear());
      app_clock_setTime = false;
    }
    app_clock_showDose = !app_clock_showDose;
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    if (app_clock_setTime)
    {
      app_clock_setTime_minute = (app_clock_setTime_minute + 1) % 60;
      retVal = true;
    }
  }

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    if (app_clock_setTime)
    {
      app_clock_setTime_hour = (app_clock_setTime_hour + 1) % 24;
      retVal = true;
    }
    else
    {
      // Exit app
      initialized = false;
      serviceRelease(SERVICE_DOSIMETRY);
      runningApp = runningApp->parent;
      retVal = true;
    }
  }

  return retVal;
}

void app_clock_draw()
{
  if (app_clock_setTime)
  {
    // Display time set screen
      u8g2.setFont(u8g2_font_logisoso32_tn);
      sprintf(printBuf, "%02d.%02d", app_clock_setTime_hour, app_clock_setTime_minute);
      u8g2.drawStr(0, 32, printBuf);
      u8g2.setFont(u8g2_font_t0_11b_tr);
      u8g2.drawStr(104, 32, "SET");
  }
  else
  {
    u8g2.setFont(u8g2_font_logisoso32_tn);
    u8g2.drawStr(0, 32, rtc.getTime("%H:%M").c_str());

    if (app_clock_showDose) app_clock_show_usv();
    else app_clock_show_cpm();
  }
}

void app_clock_show_cpm()
{
  u8g2.setFont(u8g2_font_helvB14_tf);
  if (serviceCpm < 1000)
  {
    sprintf(printBuf, "%3d", (uint16_t)serviceCpm);
    u8g2.drawStr(98, 13, printBuf);
    u8g2.setFont(u8g2_font_t0_11b_tr);
    u8g2.drawStr(104, 22, "CPM");
  }
  else if (serviceCpm < 10000)
  {
    sprintf(printBuf, "%.2f", serviceCpm/1000.0);
    u8g2.drawStr(94, 13, printBuf);
    u8g2.setFont(u8g2_font_t0_11b_tr);
    u8g2.drawStr(100, 22, "kCPM");
  }
  else if (serviceCpm < 100000)
  {
    sprintf(printBuf, "%.1f", serviceCpm/1000.0);
    u8g2.drawStr(94, 13, printBuf);
    u8g2.setFont(u8g2_font_t0_11b_tr);
    u8g2.drawStr(100, 22, "kCPM");
  }
  else if (serviceCpm < 1000000)
  {
    sprintf(printBuf, "%3d", (uint16_t)(serviceCpm/1000));
    u8g2.drawStr(98, 13, printBuf);
    u8g2.setFont(u8g2_font_t0_11b_tr);
    u8g2.drawStr(100, 22, "kCPM");
  }
  else
  {
    sprintf(printBuf, "%.2f", serviceCpm/1000000.0);
    u8g2.drawStr(94, 13, printBuf);
    u8g2.setFont(u8g2_font_t0_11b_tr);
    u8g2.drawStr(100, 22, "MCPM");
  }
}


void app_clock_show_usv()
{
  u8g2.setFont(u8g2_font_t0_11b_tr);
  u8g2.drawStr(97, 22, "uSv/h");

  u8g2.setFont(u8g2_font_helvB14_tf);
  if (serviceUsv < 10)
  {
    sprintf(printBuf, "%.2f", serviceUsv);
    u8g2.drawStr(94, 13, printBuf);
  }
  else
  {
    sprintf(printBuf, "%3d", (uint16_t)serviceUsv);
    u8g2.drawStr(98, 13, printBuf);
  }
}

