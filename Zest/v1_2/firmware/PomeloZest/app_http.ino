#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"

#include "qrcode.h"

// Data file with all web code
#include "app_http_files.h"

bool app_http_QR;

const char app_http_n42File[] PROGMEM = "<?xml version=\"1.0\"?><?xml-model href=\"http://physics.nist.gov/N42/2011/schematron/n42.sch\" type=\"application/xml\" schematypens=\"http://purl.oclc.org/dsdl/schematron\"?><RadInstrumentData xmlns=\"http://physics.nist.gov/N42/2011/N42\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://physics.nist.gov/N42/2011/N42 http://physics.nist.gov/N42/2011/n42.xsd\" n42DocUUID=\"d72b7fa7-4a20-43d4-b1b2-7e3b8c6620c1\"><RadInstrumentInformation id=\"RadInstrumentInformation-1\"><RadInstrumentManufacturerName>Pomelo</RadInstrumentManufacturerName><RadInstrumentModelName>Core</RadInstrumentModelName><RadInstrumentClassCode>Radionuclide Identifier</RadInstrumentClassCode><RadInstrumentVersion><RadInstrumentComponentName>Hardware</RadInstrumentComponentName><RadInstrumentComponentVersion>1.2</RadInstrumentComponentVersion></RadInstrumentVersion></RadInstrumentInformation><RadDetectorInformation id=\"RadDetectorInformation-1\"><RadDetectorCategoryCode>Gamma</RadDetectorCategoryCode><RadDetectorKindCode>CsI(Tl)</RadDetectorKindCode></RadDetectorInformation><EnergyCalibration id=\"EnergyCalibration-1\"><CoefficientValues>%C0% %C1% %C2%</CoefficientValues></EnergyCalibration> <RadMeasurement id=\"RadMeasurement-1\"><MeasurementClassCode>Foreground</MeasurementClassCode><StartDateTime>%DATETIME%</StartDateTime><RealTimeDuration>PT%DURATION%S</RealTimeDuration><Spectrum id=\"RadMeasurement-1Spectrum-1\" radDetectorInformationReference=\"RadDetectorInformation-1\" energyCalibrationReference=\"EnergyCalibration-1\"> <LiveTimeDuration>PT%LIVE%S</LiveTimeDuration><ChannelData compressionCode=\"None\">%SPECTRUM%</ChannelData> </Spectrum></RadMeasurement>	</RadInstrumentData>";
const char app_http_histJson[] PROGMEM = "{\"counts\":%COUNTS%,\"time\":%TIME%,\"histo\":[%DATA%]}";

String *app_http_jsonData;

AsyncWebServer app_http_webServer(80);


bool app_http_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;

  if (!initialized)
  {
    app_http_QR = false;
    app_http_jsonData = new String();
    app_http_jsonData->reserve(16384);
    serviceRequest(SERVICE_SPECTROSCOPY);
    initialized = true;
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    if (!wifi_connected)
    {
      wifiConnectSTA();
      if (wifi_connected) app_http_startServer();
    }
    else
    {
      app_http_QR = !app_http_QR;
    }
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    if (!wifi_connected)
    {
      wifiConnectAP();
      if (wifi_connected) app_http_startServer();
    }
  }

  app_http_render();

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    if (wifi_connected)
    {
      app_http_stopServer();
      wifiDisconnect();
    }
    delete app_http_jsonData;
    initialized = false;
    serviceRelease(SERVICE_SPECTROSCOPY);
    runningApp = runningApp->parent;
    return false;
  }

  return false;
}

bool app_http_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_SPECTROSCOPY)) != 0)
  {
    wifiCheck();
    app_http_render();
  }

  return true;
}

void app_http_render()
{
  if (wifi_reconnecting)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,30,"WiFi dropped");
    u8g2.drawStr(20,20,"reconnecting");
    sprintf(printBuf, "%d s...", wifi_tReconnect);
    u8g2.drawStr(0,30,printBuf);
    u8g2.sendBuffer();
  }
  else if (!app_http_QR)
  {
    // Show human readable text
    if (wifi_connected)
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      if (wifi_AP)
      {
        sprintf(printBuf, "P-%02X%02X, pomelopw", wifi_MAC[4], wifi_MAC[5]);
        u8g2.drawStr(0,10,printBuf);
      }
      else 
      {
        sprintf(printBuf, "Joined SSID: %s", wifi_ssid);
        u8g2.drawStr(0,10,printBuf);
      }
      
      if (wifi_mdns)
      {
        sprintf(printBuf, "http://%s.local", wifi_mdns_name.c_str());
      }
      else
      {
        sprintf(printBuf, "http://%s", wifi_IP.toString().c_str());
      }
      
      u8g2.drawStr(0,20,printBuf);
      u8g2.drawStr(57,30,"QR");
      u8g2.sendBuffer();
    }
    else
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0,10,"WiFi not connected");
      u8g2.drawStr(100,20,"Hot");
      u8g2.drawStr(98,30,"spot");
      if (wifi_ssid != "")
      {
        u8g2.drawLine(95, 11, 95, 32);
        u8g2.drawStr(93 - u8g2.getStrWidth(wifi_ssid.c_str()),20,wifi_ssid.c_str());
        u8g2.drawStr(48,30,"Connect");
      }
      u8g2.sendBuffer();
    }
  }
  else
  {
    // Show QR codes
    if (wifi_connected)
    {
      QRCode qrcode;
      uint8_t qrcodeBytes[qrcode_getBufferSize(3)];
      uint8_t strW;

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);

      if (wifi_AP)
      {
        u8g2.drawStr(33,10,"WiFi");
        u8g2.drawStr(33,20,"<---");
        sprintf(printBuf, "WIFI:S:P-%02X%02X;T:WPA;P:pomelopw;;", wifi_MAC[4], wifi_MAC[5]);
        qrcode_initText(&qrcode, qrcodeBytes, 3, ECC_QUARTILE, printBuf);
        for (uint8_t y = 0; y < qrcode.size; y++) {
          for (uint8_t x = 0; x < qrcode.size; x++) {
            if (qrcode_getModule(&qrcode, x, y)) u8g2.drawPixel(x + 1, y + 1);
          }
        }
      }

      u8g2.drawStr(76,10,"  UI");
      u8g2.drawStr(76,20,"--->");
      
      u8g2.drawStr(50,30,"Text");

      if (wifi_mdns)
      {
        strW = sprintf(printBuf, "HTTP://%s.LOCAL", wifi_mdns_name.c_str());
      }
      else
      {
        strW = sprintf(printBuf, "HTTP://%s", wifi_IP.toString().c_str());
      }
      if (strW <= 24)
      {
        qrcode_initText(&qrcode, qrcodeBytes, 3, ECC_HIGH, printBuf);
      }
      else
      {
        qrcode_initText(&qrcode, qrcodeBytes, 3, ECC_QUARTILE, printBuf);
      }

      for (uint8_t y = 0; y < qrcode.size; y++) {
        for (uint8_t x = 0; x < qrcode.size; x++) {
          if (qrcode_getModule(&qrcode, x, y)) u8g2.drawPixel(x + 98, y + 1);
        }
      }

      u8g2.sendBuffer();
    }
  }
}


int32_t app_http_maySleepFor()
{
  // Do not go to sleep while WiFi is running
  if (wifi_connected) return 0;
  else return 1000000; // just a long enough time to make it not matter
}


String app_http_processorN42(const String& var)
{
  static uint16_t i;
  if(var == "C0")
  {
    sprintf(printBuf, "%8e", serviceHistEcal[0]);
    return String(printBuf);
  }
  if(var == "C1")
  {
    sprintf(printBuf, "%8e", serviceHistEcal[1]);
    return String(printBuf);
  }
  if(var == "C2")
  {
    sprintf(printBuf, "%8e", serviceHistEcal[2]);
    return String(printBuf);
  }
  if(var == "DATETIME")
  {
    // 2003-11-22T23:45:19-07:00
    sprintf(printBuf, "%s", rtc.getTime("%Y-%m-%dT%H:%M:%S").c_str());
    return String(printBuf);
  }
  if(var == "DURATION")
  {
    sprintf(printBuf, "%d", serviceHistTime);
    return String(printBuf);
  }
  if(var == "LIVE")
  {
    sprintf(printBuf, "%.2f", serviceHistTime*1.0 - serviceHistCounts*0.00007);  // About 70us dead time per event
    return String(printBuf);
  }
  if(var == "SPECTRUM")
  {
    *app_http_jsonData = "";
    for (i = 0; i < 1023; i++)
    {
      sprintf(printBuf, "%d ", serviceHist[i]);
      app_http_jsonData->concat(printBuf);
    }
    sprintf(printBuf, "%d", serviceHist[1023]);
    app_http_jsonData->concat(printBuf);
    return *app_http_jsonData;
  }
  return String();
}

String app_http_processorHist(const String& var)
{
  static uint16_t i;

  if(var == "DATA")
  {
    *app_http_jsonData = "";
    for (i = 0; i < 1023; i++)
    {
      sprintf(printBuf, "%d,", serviceHist[i]);
      app_http_jsonData->concat(printBuf);
    }
    sprintf(printBuf, "%d", serviceHist[1023]);
    app_http_jsonData->concat(printBuf);
    return *app_http_jsonData;
  }
  if(var == "COUNTS")
  {
    sprintf(printBuf, "%d", serviceHistCounts);
    return String(printBuf);
  }
  if(var == "TIME")
  {
    sprintf(printBuf, "%d", serviceHistTime);
    return String(printBuf);
  }
  return String();
}

void app_http_stopServer()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,20, "Shutting down server...");
  u8g2.sendBuffer();
  wifiStopMdns();
  delay(500);
  app_http_webServer.reset();
  app_http_webServer.end();
  delay(500);
  tcpCleanup();
}


void app_http_startServer()
{
  wifiStartMdns();

  // Static pages, text
  app_http_webServer.on("/uPlot.iife.min.js", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/javascript", uPlot_iife_min_js); });
  app_http_webServer.on("/uPlot.min.css", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/css", uPlot_min_css); });
  app_http_webServer.on("/browserconfig.xml", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", browserconfig_xml); });
  app_http_webServer.on("/site.webmanifest", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", site_webmanifest); });

  // Static resources, binary
  app_http_webServer.on("/android-chrome-192x192.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", android_chrome_192x192_png, 14447); request->send(response); });
  app_http_webServer.on("/android-chrome-512x512.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", android_chrome_512x512_png, 40687); request->send(response); });
  app_http_webServer.on("/apple-touch-icon.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", apple_touch_icon_png, 9007); request->send(response); });
  app_http_webServer.on("/favicon-16x16.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", favicon_16x16_png, 1106); request->send(response); });
  app_http_webServer.on("/favicon-32x32.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", favicon_32x32_png, 2151); request->send(response); });
  app_http_webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/vnd.microsoft.icon", favicon_ico, 15086); request->send(response); });
  app_http_webServer.on("/mstile-144x144.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_144x144_png, 8925); request->send(response); });
  app_http_webServer.on("/mstile-150x150.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_150x150_png, 8717); request->send(response); });
  app_http_webServer.on("/mstile-310x150.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_310x150_png, 9366); request->send(response); });
  app_http_webServer.on("/mstile-310x310.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_310x310_png, 17934); request->send(response); });
  app_http_webServer.on("/mstile-70x70.png", HTTP_GET, [](AsyncWebServerRequest *request) { AsyncWebServerResponse *response = request->beginResponse_P(200, "image/png", mstile_70x70_png, 6320); request->send(response); });
  app_http_webServer.on("/safari-pinned-tab.svg", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", safari_pinned_tab_svg); });

  // Dynamic pages start here: index, wifi, time, spectrum
  app_http_webServer.on("/spectrum.n42", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "application/octet-stream", app_http_n42File, app_http_processorN42); });

  app_http_webServer.on("/wifi.htm", HTTP_ANY, [](AsyncWebServerRequest *request)
  {
    int params = request->params();
    bool update;
    update = false;
    for(int i=0;i<params;i++)
    {
      const AsyncWebParameter* p = request->getParam(i);
      if(p->isPost())
      {
        update = true;
        if (p->name() == "ssid") wifi_ssid = p->value();
        if (p->name() == "pass") wifi_pass = p->value();
      }
    }
    if (update)
    {
      preferences.begin("pomeloZest", false);
      preferences.putString("ssid", wifi_ssid); 
      preferences.putString("password", wifi_pass);
      preferences.end();
    }
    request->send_P(200, "text/html", wifi_htm); 
  });

  app_http_webServer.on("/time.htm", HTTP_ANY, [](AsyncWebServerRequest *request)
  {
    int params = request->params();
    bool update;
    update = false;
    uint16_t yr, mo, da, ho, mi;
    yr = mo = da = ho = mi = -1;
    for(int i=0;i<params;i++)
    {
      const AsyncWebParameter* p = request->getParam(i);
      if(p->isPost())
      {
        update = true;
        if (p->name() == "year") yr = p->value().toInt();
        if (p->name() == "month") mo = p->value().toInt();
        if (p->name() == "day") da = p->value().toInt();
        if (p->name() == "hour") ho = p->value().toInt();
        if (p->name() == "minute") mi = p->value().toInt();
      }
    }
    if (update && yr != -1 && mo != -1 && da != -1 && ho != -1 && mi != -1)
    {
      rtc.setTime(0, mi, ho, da, mo, yr);
    }
    request->send_P(200, "text/html", time_htm); 
  });

  app_http_webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) 
  {
    if (request->hasParam("restart"))
    {
      if (request->getParam("restart")->value().toInt() == 1)
      {
        coreCommand("x");
        for (int i = 0; i < 1024; i++)
        {
          serviceHist[i] = 0;
        }
      }
    }
    if (request->hasParam("stop"))
    {
      if (request->getParam("stop")->value().toInt() == 1) coreCommand("z");
    }
    request->send_P(200, "text/html", index_htm, app_http_processorN42); 
  });
  app_http_webServer.on("/hist.json", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "application/json", app_http_histJson, app_http_processorHist); });
  app_http_webServer.begin();
}

