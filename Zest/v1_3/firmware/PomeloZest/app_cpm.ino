
bool app_cpm_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    serviceRequest(SERVICE_DOSIMETRY);
    initialized = true;
    retVal = true;
  }

  if ((serviceFlags & (1 << SERVICE_DOSIMETRY)) != 0)
  {
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_DOSIMETRY);
    runningApp = runningApp->parent;
    retVal = true;
  }

  return retVal;
}

void app_cpm_draw()
{
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
}
