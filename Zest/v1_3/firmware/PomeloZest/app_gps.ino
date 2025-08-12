#include "FS.h"
#include "SD.h"
#include "SPI.h"

bool app_gps_sdOk, app_gps_fileLogging;

File app_gps_file;
uint32_t app_gps_fileNumber;
uint32_t app_gps_fileEntry;
char app_gps_dir[80];


bool app_gps_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    app_gps_sdOk = false;
    app_gps_fileLogging = false;
    serviceRequest(SERVICE_GPS);
    initialized = true;
    retVal = true;
  }

  if (app_gps_sdOk && app_gps_fileLogging)
  {
    // We only do take actions if we need to perform logging

    if ((serviceFlags & (1 << SERVICE_SPECTROSCOPY)) != 0)
    {
      coreCommand("x");
      app_gps_logSpectrum();
      retVal = true;
    }

    if ((serviceFlags & (1 << SERVICE_GPS)) != 0)
    {
      // Ask for a histogram such that it will get logged
      coreCommand("h");
      noSleepUntilWatermark(tNow + 200); // Make sure we catch histogram
    }
  }


  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    if (app_gps_sdOk)
    {
      if (app_gps_fileLogging)
      {
        // Stop logging
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        app_gps_file.close();
        app_gps_fileLogging = false;
        u8g2.drawStr(0,10,"STOP LOGGING");
        u8g2.sendBuffer();
        delay(500);
      }
      else
      {
        // Start logging
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        // Find first unused directory
        for (app_gps_fileNumber = 0; app_gps_fileNumber < 0xFFFFFFFF; app_gps_fileNumber++)
        {
          sprintf(app_gps_dir, "/G%07lu", app_gps_fileNumber);
          if (!SD.exists(app_gps_dir)) break;
        }
        if (SD.mkdir(app_gps_dir))
        {
          app_gps_fileNumber = 0;
          app_gps_fileEntry = 0;
          sprintf(printBuf, "/%s/%08lu.csv", app_gps_dir, app_gps_fileNumber);
          app_gps_file = SD.open(printBuf, FILE_WRITE);
          if (app_gps_file)
          {
            u8g2.drawStr(0,10,"START LOGGING");
            app_gps_fileLogging = true;
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
    if (app_gps_sdOk)
    {
        if (app_gps_fileLogging)
        {
          app_gps_file.close();
          app_gps_fileLogging = false;
        }
        SD.end();
        app_gps_sdOk = false;
    }
    else
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      if(SD.begin())
      {
        u8g2.drawStr(0,10,"SD card OK");
        app_gps_sdOk = true;
        app_gps_fileLogging = false;
      }
      else
      {
        u8g2.drawStr(0,10,"SD card failed");
        app_gps_sdOk = false;
        app_gps_fileLogging = false;
      }
      u8g2.sendBuffer();
      delay(500);
    }
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    if (app_gps_sdOk)
    {
        if (app_gps_fileLogging)
        {
          app_gps_file.close();
          app_gps_fileLogging = false;
        }
        SD.end();
        app_gps_sdOk = false;
    }
    initialized = false;
    serviceRelease(SERVICE_GPS);
    runningApp = runningApp->parent;
    retVal = true;
  }

  return retVal;
}

void app_gps_draw()
{
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(printBuf, "GPS: %02d:%02d, F: %08lu", GPS.hour, GPS.minute, app_gps_fileNumber);
  u8g2.drawStr(0,10, printBuf);
  sprintf(printBuf, "G: %03lu", app_gps_fileEntry);
  u8g2.drawStr(0,20, printBuf);

  if (app_gps_sdOk)
  {
    u8g2.drawLine(73, 11, 73, 32);
    u8g2.drawStr(75,20,"Unmount");
    u8g2.drawStr(95,30,"SD");
    u8g2.drawStr(50,20,"Log");
    if (app_gps_fileLogging)
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



void app_gps_logSpectrum()
{
  JsonDocument doc;
  static float lat, lon;
  static uint16_t i;

  lat = GPS.latitudeDegrees;
  lon = GPS.longitudeDegrees;
  if (GPS.lat == 'S') lat = -lat;
  if (GPS.lon == 'W') lon = -lon;

  doc["timestamp"]["year"] = GPS.year;
  doc["timestamp"]["month"] = GPS.month;
  doc["timestamp"]["day"] = GPS.day;
  doc["timestamp"]["hour"] = GPS.hour;
  doc["timestamp"]["minute"] = GPS.minute;
  doc["timestamp"]["seconds"] = GPS.seconds;
  
  doc["location"]["lat"] = lat;
  doc["location"]["lon"] = lon;
  doc["location"]["speed"] = GPS.speed;
  doc["location"]["fix"] = (int)GPS.fix;
  doc["location"]["fixQ"] = (int)GPS.fixquality;
  doc["location"]["angle"] = GPS.angle;
  doc["location"]["alt"] = GPS.altitude;
  doc["location"]["nSat"] = (int)GPS.satellites;
  doc["location"]["hdop"] = GPS.HDOP;

  doc["spectrum"]["time"] = serviceHistTime;
  doc["spectrum"]["counts"] = serviceHistCounts;
  doc["spectrum"]["temperature"] = serviceCoreTemp;
  for (i = 0; i < 1024; i++)
  {
    doc["spectrum"]["hist"][i] = serviceHist[i];
  }

  serializeJson(doc, buf);

  if (app_gps_sdOk && app_gps_fileLogging)
  {
    app_gps_file.println(buf);

    app_gps_fileEntry++;

    if (app_gps_fileEntry > 100)
    {
      app_gps_fileEntry = 0;
      app_gps_file.close();

      app_gps_fileNumber++;
      sprintf(printBuf, "/%s/%08lu.csv", app_gps_dir, app_gps_fileNumber);
      app_gps_file = SD.open(printBuf, FILE_WRITE);
      if (app_gps_file)
      {
        u8g2.drawStr(0,10,"New file");
      }
      else
      {
        u8g2.drawStr(0,10,"Cannot create new file");
        app_gps_fileLogging = false;
      }
    }
  }
}


int32_t app_gps_maySleepFor()
{
  // Do not go to sleep while we're logging
  if (app_gps_sdOk && app_gps_fileLogging) return 0;
  else return 1000000; // just a long enough time to not make it matter
}

