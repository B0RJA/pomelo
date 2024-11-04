
bool app_cpm_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;

  if (!initialized)
  {
    serviceRequest(SERVICE_DOSIMETRY);
    initialized = true;
  }

  app_cpm_render();
  
  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_DOSIMETRY);
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

bool app_cpm_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_DOSIMETRY)) != 0)
  {
    app_cpm_render();
  }

  return true;
}

void app_cpm_render()
{
  u8g2.clearBuffer();
  drawHistory(bufHistoryCpm);

  u8g2.setFont(u8g2_font_t0_11b_tr);
  if (serviceCpm < 1000)
  {
    u8g2.drawStr(95, 32, "CPM");
    u8g2.setFont(u8g2_font_fub20_tf);
    sprintf(printBuf, "%3d", (uint16_t)serviceCpm);
    u8g2.drawStr(83, 22, printBuf);
  }
  else if (serviceCpm < 10000)
  {
    u8g2.drawStr(92, 32, "kCPM");
    u8g2.setFont(u8g2_font_fub20_tf);
    sprintf(printBuf, "%.2f", serviceCpm/1000.0);
    u8g2.drawStr(73, 22, printBuf);
  }
  else if (serviceCpm < 100000)
  {
    u8g2.drawStr(92, 32, "kCPM");
    u8g2.setFont(u8g2_font_fub20_tf);
    sprintf(printBuf, "%.1f", serviceCpm/1000.0);
    u8g2.drawStr(73, 22, printBuf);
  }
  else if (serviceCpm < 1000000)
  {
    u8g2.drawStr(92, 32, "kCPM");
    u8g2.setFont(u8g2_font_fub20_tf);
    sprintf(printBuf, "%3d", (uint16_t)(serviceCpm/1000));
    u8g2.drawStr(83, 22, printBuf);
  }
  else
  {
    u8g2.drawStr(92, 32, "MCPM");
    u8g2.setFont(u8g2_font_fub20_tf);
    sprintf(printBuf, "%.2f", serviceCpm/1000000.0);
    u8g2.drawStr(73, 22, printBuf);
  }

  u8g2.sendBuffer();  
}
