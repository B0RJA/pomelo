#define APP_TCP_MAX_CLIENTS   8
#define APP_TCP_BUF_LEN       128

uint16_t app_tcp_txBufN;
uint8_t app_tcp_txBuf[APP_TCP_BUF_LEN];

WiFiServer app_tcp_server(9378);
WiFiClient *app_tcp_client[APP_TCP_MAX_CLIENTS];

uint8_t app_tcp_nClients;

bool app_tcp_updateUI(uint8_t buttonFlags)
{
  static bool initialized = false;
  static uint8_t i, j;
  static WiFiClient client;
  static uint16_t bytesWritten;
  static uint32_t t0;

  if (!initialized)
  {
    for (i = 0; i < APP_TCP_MAX_CLIENTS; i++) app_tcp_client[i] = NULL;
    app_tcp_nClients = 0;
    app_tcp_txBufN = 0;
    serviceRequest(SERVICE_DATASTREAM);
    initialized = true;
    app_tcp_render();
  }

  if (wifi_connected)
  {
    client = app_tcp_server.available();
    if (client)
    {
      // new client. Put it somewhere.
      for (i = 0; i < APP_TCP_MAX_CLIENTS; i++)
      {
        if (app_tcp_client[i] == NULL)
        {
          app_tcp_client[i] = new WiFiClient(client);
          //app_tcp_client[i]->write("\377\375\042\377\373\001",6);   // Initiate telnet mode in a way that makes sense?
          app_tcp_nClients++;
          break;
        }
      }
      if (i == APP_TCP_MAX_CLIENTS)
      {
        // No space for another one, disconnect
        client.stop();
      }
      app_tcp_render();
    }

    for (i = 0; i < APP_TCP_MAX_CLIENTS; i++)
    {
      if (app_tcp_client[i] != NULL)
      {
        if (app_tcp_client[i]->connected())
        {
          if (app_tcp_client[i]->available())
          {
            // We have some data. Send it to Core
            j = 0;
            while ((app_tcp_client[i]->available()) && (j < 63))
            {
              printBuf[j] = app_tcp_client[i]->read();
              j++;
            }
            printBuf[j] = 0;
            coreCommand(printBuf);
          }

          if (app_tcp_txBufN != 0)
          {
            // We have data from core, send to client
            bytesWritten = 0;
            t0 = millis();
            while (bytesWritten != app_tcp_txBufN)
            {
              bytesWritten += app_tcp_client[i]->write(&app_tcp_txBuf[bytesWritten], app_tcp_txBufN - bytesWritten);
              if (millis() - t0 > 1000) break; // timeout after 1s
            }
          }
        }
        else
        {
          // Disconnected, remove
          app_tcp_client[i]->stop();
          delete app_tcp_client[i];
          app_tcp_client[i] = NULL;
          app_tcp_nClients--;
          app_tcp_render();
        }
      }
    }
  }
  app_tcp_txBufN = 0;


  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    if (!wifi_connected)
    {
      wifiConnectSTA();
      if (wifi_connected) app_tcp_startServer();
    }
    app_tcp_render();
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    if (!wifi_connected)
    {
      wifiConnectAP();
      if (wifi_connected) app_tcp_startServer();
    }
    app_tcp_render();
  }

  // TCP updateUI runs constantly to keep
  // looking for client interaction. Check for
  // WiFi disconnect
  wifiCheck();

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    if (wifi_connected)
    {
      // Disconnect all clients
      for (i = 0; i < APP_TCP_MAX_CLIENTS; i++)
      {
        if (app_tcp_client[i] != NULL)
        {
          // Disconnected, remove
          app_tcp_client[i]->stop();
          delete app_tcp_client[i];
          app_tcp_client[i] = NULL;
          app_tcp_nClients--;
        }
      }
      app_http_stopServer();
      wifiDisconnect();
    }
    initialized = false;
    serviceRelease(SERVICE_DATASTREAM);
    runningApp = runningApp->parent;
    return false;
  }

  return false;
}


bool app_tcp_updateService(uint8_t serviceFlags)
{
  if ((serviceFlags & (1 << SERVICE_DATASTREAM)) != 0)
  {
    if (app_tcp_txBufN < APP_TCP_BUF_LEN)
    {
      app_tcp_txBuf[app_tcp_txBufN] = serialRead();
      app_tcp_txBufN++;
      return true;
    }
    else
    {
      return false;
    }
  }
  return true;
}

void app_tcp_render()
{
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
      sprintf(printBuf, "%s.local:9378", wifi_mdns_name.c_str());
    }
    else
    {
      sprintf(printBuf, "%s:9378", wifi_IP.toString().c_str());
    }
    u8g2.drawStr(0,20,printBuf);
    sprintf(printBuf, "Clients: %d", app_tcp_nClients);
    u8g2.drawStr(0,30,printBuf);
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


int32_t app_tcp_maySleepFor()
{
  // Do not go to sleep while WiFi is running
  if (wifi_connected) return 0;
  else return 1000000; // just a long enough time to not make it matter
}


void app_tcp_stopServer()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,20, "Shutting down server...");
  u8g2.sendBuffer();
  wifiStopMdns();
  delay(500);
  app_tcp_server.end();
  delay(500);
  tcpCleanup();
}

void app_tcp_startServer()
{
  wifiStartMdns();
  app_tcp_server.begin();
}

