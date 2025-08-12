
bool app_usv_run(uint8_t buttonFlags, uint8_t serviceFlags)
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

void app_usv_draw()
{
  drawHistory(bufHistoryUsv);

  u8g2.setFont(u8g2_font_t0_11b_tr);
  u8g2.drawStr(89, 32, "uSv/h");
  u8g2.setFont(u8g2_font_fub20_tf);
  if (serviceUsv < 10)
  {
    sprintf(printBuf, "%.2f", serviceUsv);
    u8g2.drawStr(73, 22, printBuf);
  }
  else
  {
    sprintf(printBuf, "%3d", (uint16_t)serviceUsv);
    u8g2.drawStr(83, 22, printBuf);
  }
}
