#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define APP_LOG_FILE_INTERVAL_S 60

bool app_log_sdOk, app_log_fileLogging;
uint32_t app_log_tLog;

File app_log_file;
uint32_t app_log_fileNumber;
uint32_t app_log_fileEntry;
char app_log_dir[80];


bool app_log_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    app_log_sdOk = false;
    app_log_fileLogging = false;
    app_log_tLog = 0;
    serviceRequest(SERVICE_TICK);
    initialized = true;
    retVal = true;
  }

  if (app_log_sdOk && app_log_fileLogging)
  {
    // We only do take actions if we need to perform logging

    if ((serviceFlags & (1 << SERVICE_SPECTROSCOPY)) != 0)
    {
      coreCommand("x");
      app_log_logSpectrum();
      retVal = true;
    }

    if ((serviceFlags & (1 << SERVICE_TICK)) != 0)
    {
      if (app_log_tLog == 0)
      {
        // Ask for a histogram such that it will get logged
        coreCommand("h");
        noSleepUntilWatermark(tNow + 200); // Make sure we catch histogram
        app_log_tLog = APP_LOG_FILE_INTERVAL_S;
      }
      else
      {
        app_log_tLog--;
      }
    }
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    if (app_log_sdOk)
    {
      if (app_log_fileLogging)
      {
        // Stop logging
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        app_log_file.close();
        app_log_fileLogging = false;
        u8g2.drawStr(0,10,"STOP LOGGING");
        u8g2.sendBuffer();
        delay(500);
      }
      else
      {
        // Start logging
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        app_log_tLog = 0;
        // Find first unused directory
        for (app_log_fileNumber = 0; app_log_fileNumber < 0xFFFFFFFF; app_log_fileNumber++)
        {
          sprintf(app_log_dir, "/%08lu", app_log_fileNumber);
          if (!SD.exists(app_log_dir)) break;
        }
        if (SD.mkdir(app_log_dir))
        {
          app_log_fileNumber = 0;
          app_log_fileEntry = 0;
          sprintf(printBuf, "/%s/%08lu.csv", app_log_dir, app_log_fileNumber);
          app_log_file = SD.open(printBuf, FILE_WRITE);
          if (app_log_file)
          {
            u8g2.drawStr(0,10,"START LOGGING");
            app_log_fileLogging = true;
          }
          else
          {
            u8g2.drawStr(0,10,"Cannot create file");
          }
        }
        else
        {
          u8g2.drawStr(0,10,"Cannot create dir");
        }
        u8g2.sendBuffer();
        delay(500);
      }
    }
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    if (app_log_sdOk)
    {
        if (app_log_fileLogging)
        {
          app_log_file.close();
          app_log_fileLogging = false;
        }
        SD.end();
        app_log_sdOk = false;
    }
    else
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      if(SD.begin())
      {
        u8g2.drawStr(0,10,"SD card OK");
        app_log_sdOk = true;
        app_log_fileLogging = false;
      }
      else
      {
        u8g2.drawStr(0,10,"SD card failed");
        app_log_sdOk = false;
        app_log_fileLogging = false;
      }
      u8g2.sendBuffer();
      delay(500);
    }
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    if (app_log_sdOk)
    {
        if (app_log_fileLogging)
        {
          app_log_file.close();
          app_log_fileLogging = false;
        }
        SD.end();
        app_log_sdOk = false;
    }
    initialized = false;
    serviceRelease(SERVICE_TICK);
    runningApp = runningApp->parent;
    retVal = true;
  }

  return retVal;
}

void app_log_draw()
{
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(printBuf, "RTC: %02d:%02d, F: %08lu", rtc.getHour(true), rtc.getMinute(), app_log_fileNumber);
  u8g2.drawStr(0,10, printBuf);
  sprintf(printBuf, "L: %03lu", app_log_fileEntry);
  u8g2.drawStr(0,20, printBuf);

  if (app_log_sdOk)
  {
    u8g2.drawLine(73, 11, 73, 32);
    u8g2.drawStr(75,20,"Unmount");
    u8g2.drawStr(95,30,"SD");
    u8g2.drawStr(50,20,"Log");
    if (app_log_fileLogging)
    {
      u8g2.drawStr(47,30,"Stop");
    }
    else
    {
      u8g2.drawStr(45,30,"Start");
    }
  }
  else
  {
    u8g2.drawStr(90,20,"Mount");
    u8g2.drawStr(100,30,"SD");
  }
}


void app_log_logSpectrum()
{
  JsonDocument doc;
  //DeserializationError error = deserializeJson(doc, payload);
  static uint16_t i;

  // Now save log to currently open file
  doc["timestamp"]["year"] = rtc.getYear();
  doc["timestamp"]["month"] = rtc.getMonth() + 1;
  doc["timestamp"]["day"] = rtc.getDay();
  doc["timestamp"]["hour"] = rtc.getHour(true);
  doc["timestamp"]["minute"] = rtc.getMinute();
  doc["timestamp"]["seconds"] = rtc.getSecond();

  doc["spectrum"]["time"] = serviceHistTime;
  doc["spectrum"]["counts"] = serviceHistCounts;
  doc["spectrum"]["temperature"] = serviceCoreTemp;
  for (i = 0; i < 1024; i++)
  {
    doc["spectrum"]["hist"][i] = serviceHist[i];
  }

  serializeJson(doc, buf);

  if (app_log_sdOk && app_log_fileLogging)
  {
    app_log_file.println(buf);

    app_log_fileEntry++;

    if (app_log_fileEntry > 100)
    {
      app_log_fileEntry = 0;
      app_log_file.close();

      app_log_fileNumber++;
      sprintf(printBuf, "/%s/%08lu.csv", app_log_dir, app_log_fileNumber);
      app_log_file = SD.open(printBuf, FILE_WRITE);
      if (app_log_file)
      {
        u8g2.drawStr(0,10,"New file");
      }
      else
      {
        u8g2.drawStr(0,10,"Cannot create new file");
        app_log_fileLogging = false;
      }
    }
  }
}

