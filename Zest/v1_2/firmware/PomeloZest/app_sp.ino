bool app_sp_log;


bool app_sp_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;

  if (!initialized)
  {
    app_sp_log = true;
    initialized = true;
    serviceRequest(SERVICE_SPECTROSCOPY);
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    app_sp_log = !app_sp_log;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    coreCommand("x");
    uint16_t i;
    for (i = 0; i < 1024; i++)
    {
      serviceHist[i] = 0;
    }
    serviceHistCounts = 0;
    serviceHistTime = 0;
  }

  drawSpectrum(app_sp_log);

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_SPECTROSCOPY);
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

bool app_sp_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_SPECTROSCOPY)) != 0)
  {
    drawSpectrum(app_sp_log);
  }

  return true;
}

