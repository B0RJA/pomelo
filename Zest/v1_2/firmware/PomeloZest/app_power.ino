
bool app_power_updateUI(uint8_t buttonFlags)
{
  float vbat;
  static bool initialized = false;

  if (!initialized)
  {
    serviceRequest(SERVICE_TICK);
    initialized = true;
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    coreCommand("z");
    zest_deepsleep();
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    zest_deepsleep();
  }

  app_power_display();
  
  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_TICK);
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

bool app_power_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_TICK)) != 0)
  {
    app_power_display();
  }
  return true;
}


void app_power_display()
{
  float vbat;

  vbat = vbat_read();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, rtc.getTime("%Y.%m.%d").c_str());
  u8g2.drawStr(0, 20, rtc.getTime("%H:%M:%S").c_str());
  sprintf(printBuf, "Bat: %.2f V", vbat);
  u8g2.drawStr(64, 10, printBuf);

  u8g2.drawStr(45, 30,"Pw Off");
  u8g2.drawStr(95,30,"Sleep");
  u8g2.sendBuffer();
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


