
bool app_power_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    initialized = true;
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    // Switch off Core
    coreCommand("z");
    if (app_power.parent != &app_menu)
    {
      // Tell currently running application to quit grcefullt by sending a button press
      app_power.parent->run((1 << BUTTON_1), 0);
    }
    // Make sure that we come back to the app when powering back up
    runningApp = app_power.parent; 
    zest_deepsleep();
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    if (app_power.parent != &app_menu)
    {
      app_power.parent->run((1 << BUTTON_1), 0);
    }
    runningApp = app_power.parent; 
    zest_deepsleep();
  }

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    runningApp = app_power.parent;
    app_power.parent = NULL;
    retVal = true;
  }

  return retVal;
}


void app_power_draw()
{
  u8g2.drawRBox(0, 20, 128, 12, 3);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 30,"Cancel");
  u8g2.drawStr(45,30,"Pw Off");
  u8g2.drawStr(95,30,"Sleep");
  u8g2.setDrawColor(1);
}

