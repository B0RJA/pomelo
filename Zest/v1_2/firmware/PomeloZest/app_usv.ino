
bool app_usv_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;

  if (!initialized)
  {
    serviceRequest(SERVICE_DOSIMETRY);
    initialized = true;
  }

  app_usv_render();
  
  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_DOSIMETRY);
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

bool app_usv_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_DOSIMETRY)) != 0)
  {
    app_usv_render();
  }

  return true;
}

void app_usv_render()
{
  u8g2.clearBuffer();
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
  u8g2.sendBuffer();
}
