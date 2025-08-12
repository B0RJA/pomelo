bool app_sp_log;


bool app_sp_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    app_sp_log = true;
    initialized = true;
    serviceRequest(SERVICE_SPECTROSCOPY);
    retVal = true;
  }

  if ((serviceFlags & (1 << SERVICE_SPECTROSCOPY)) != 0)
  {
    retVal = true;
  }


  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    app_sp_log = !app_sp_log;
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_3_LONG)) != 0)
  {
    coreCommand("x");
    uint16_t i;
    for (i = 0; i < 1024; i++)
    {
      serviceHist[i] = 0;
    }
    serviceHistCounts = 0;
    serviceHistTime = 0;
    retVal = true;
  }


  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_SPECTROSCOPY);
    runningApp = runningApp->parent;
    retVal = true;
  }

  return retVal;
}

void app_sp_draw()
{
  drawSpectrum(app_sp_log);
}

