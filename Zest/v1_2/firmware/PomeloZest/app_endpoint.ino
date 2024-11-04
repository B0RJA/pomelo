#include <WiFiClientSecure.h>

#define APP_ENDPOINT_INTERVAL_S 3600

WiFiClientSecure app_endpoint_client;

const char* app_endpoint_root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\n" \
"-----END CERTIFICATE-----\n";


const char* app_endpoint_api_endpoint = "... xxx ...";
const char* app_endpoint_api_handler = "... xxx ...";
const char* app_endpoint_api_key = "... xxx ...";

uint32_t app_endpoint_tLog;
uint32_t app_endpoint_entries;
bool app_endpoint_logging;

bool app_endpoint_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;

  if (!initialized)
  {
    app_endpoint_tLog = 0;
    app_endpoint_entries = 0;
    app_endpoint_logging = false;
    serviceRequest(SERVICE_TICK);
    initialized = true;
  }


  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    app_endpoint_logging = !app_endpoint_logging;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    // Log now!
    coreCommand("h");
    noSleepUntil(tNow + 200); // Make sure we catch histogram
    app_endpoint_tLog = APP_ENDPOINT_INTERVAL_S;
  }

  app_endpoint_render();

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    initialized = false;
    serviceRelease(SERVICE_TICK);
    runningApp = runningApp->parent;
    return false;
  }

  return true;
}

void app_endpoint_render()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  sprintf(printBuf, "RTC: %02d:%02d, L: %d", rtc.getHour(true), rtc.getMinute(), app_endpoint_logging);
  u8g2.drawStr(0,10, printBuf);
  sprintf(printBuf, "N: %03d", app_endpoint_entries);
  u8g2.drawStr(0,20, printBuf);
  u8g2.sendBuffer();
}


bool app_endpoint_updateService(uint8_t serviceFlags)
{
  if (app_endpoint_logging)
  {
    // We only do take actions if we need to perform logging

    if ((serviceFlags & (1 << SERVICE_SPECTROSCOPY)) != 0)
    {
      app_endpoint_logSpectrum();
      app_endpoint_render();
      app_endpoint_tLog = APP_ENDPOINT_INTERVAL_S;
    }

    if ((serviceFlags & (1 << SERVICE_TICK)) != 0)
    {
      if (app_endpoint_tLog == 0)
      {
        // Ask for a histogram such that it will get logged
        coreCommand("h");
        noSleepUntil(tNow + 200); // Make sure we catch histogram
      }
      else
      {
        app_endpoint_tLog--;
      }
    }
  }

  return true;
}


void app_endpoint_logSpectrum()
{
  DynamicJsonDocument doc(32768);
  DeserializationError error;
  uint16_t i;
  uint8_t state;
  uint32_t timeOut;
  bool success;
  char localBuf[256];


  Network.macAddress(wifi_MAC);
  sprintf(printBuf, "%02X%02X%02X%02X%02X%02X", wifi_MAC[0], wifi_MAC[1], wifi_MAC[2], wifi_MAC[3], wifi_MAC[4], wifi_MAC[5]);
  doc["id"] = printBuf;

  // Now save log to currently open file
  doc["payload"]["timestamp"]["year"] = rtc.getYear();
  doc["payload"]["timestamp"]["month"] = rtc.getMonth() + 1;
  doc["payload"]["timestamp"]["day"] = rtc.getDay();
  doc["payload"]["timestamp"]["hour"] = rtc.getHour(true);
  doc["payload"]["timestamp"]["minute"] = rtc.getMinute();
  doc["payload"]["timestamp"]["seconds"] = rtc.getSecond();

  doc["payload"]["spectrum"]["time"] = serviceHistTime;
  doc["payload"]["spectrum"]["counts"] = serviceHistCounts;
  doc["payload"]["spectrum"]["temperature"] = serviceCoreTemp;
  for (i = 0; i < 1024; i++)
  {
    doc["payload"]["spectrum"]["hist"][i] = serviceHist[i];
  }

  serializeJson(doc, buf);

  success = true;
  wifiConnectSTA();
  if (wifi_connected)
  {
    app_endpoint_client.setCACert(app_endpoint_root_ca);
    app_endpoint_client.setTimeout(5000);

    if (!app_endpoint_client.connect(app_endpoint_api_endpoint, 443))
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,20, "API Connection failed");
      u8g2.sendBuffer();
      delay(500);
    }
    else
    {

      sprintf(localBuf, "GET https://%s/%s HTTP/1.0\r\n", app_endpoint_api_endpoint, app_endpoint_api_handler);
      app_endpoint_client.print(localBuf);

      sprintf(localBuf, "Host: %s\r\n", app_endpoint_api_endpoint);
      app_endpoint_client.print(localBuf);
      
      app_endpoint_client.print("User-Agent: curl/7.58.0\r\n");
      app_endpoint_client.print("Accept: */*\r\n");
      
      sprintf(localBuf, "x-api-key: %s\r\n", app_endpoint_api_key);
      app_endpoint_client.print(localBuf);
      app_endpoint_client.print("Content-Type: application/json\r\n");

      i = strlen(buf);
      sprintf(localBuf, "Content-Length: %d\r\n", i);
      app_endpoint_client.print(localBuf);
      app_endpoint_client.print("\r\n");
      app_endpoint_client.print(buf);
      app_endpoint_client.print("\r\n");

      i = 0;
      timeOut = millis();
      while (app_endpoint_client.connected() && (i < sizeof(buf) - 1))
      {
        if (millis() - timeOut > 5000) break;
        while (app_endpoint_client.available() && (i < sizeof(buf) - 1))
        {
          buf[i++] = app_endpoint_client.read();
        }
        delay(100);
      }
      buf[i] = 0;

      app_endpoint_client.stop();

      if (i == 0) success = false;
      else
      {
        i = 0;
        state = 0;
        while (buf[i] != 0)
        {
          if (buf[i] == '\r')
          {
            if ((state == 0) || (state == 2)) state++;
          }
          else if (buf[i] == '\n')
          {
            if ((state == 1) || (state == 3)) state++;
          }
          else
          {
            state = 0;
          }
          i++;
          if (state == 4) break;
        }

        error = deserializeJson(doc, &buf[i]);
        if (error) success = false;
        else
        {
          if (doc["Error"].is<JsonVariant>()) success = false;
        }
      }

    }

    if (success)
    {
      app_endpoint_entries++;
      coreCommand("x");
    }
  }
  wifiDisconnect();
}

