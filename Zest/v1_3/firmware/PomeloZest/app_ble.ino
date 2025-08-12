#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UART_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UART_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UART_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


bool app_ble_txCpm = true;
bool app_ble_deviceConnected = false;

BLEServer *app_ble_pServer;
BLEAdvertising *app_ble_pAdvertising;

BLEService *app_ble_pUartService;
BLECharacteristic *app_ble_pUartRxCharacteristic;
BLECharacteristic *app_ble_pUartTxCharacteristic;
BLE2902 *app_ble_pUartBLE2902;


class app_ble_ServerCallbacks: public BLEServerCallbacks
{
  void onConnect(BLEServer* pServer)
  {
    app_ble_deviceConnected = true;
  };
  
  void onDisconnect(BLEServer* pServer)
  {
    app_ble_deviceConnected = false;
    app_ble_pAdvertising->start();
  }
};

class app_ble_UartCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      String rxValue = pCharacteristic->getValue();
    }
};

app_ble_ServerCallbacks *app_ble_pServerCallbacks;
app_ble_UartCallbacks *app_ble_pUartCallbacks;


bool app_ble_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static bool initialized = false;
  bool retVal;

  retVal = false;

  if (!initialized)
  {
    BLEDevice::init("Pomelo Zest");
    app_ble_pServer = BLEDevice::createServer();
    app_ble_pServerCallbacks = new app_ble_ServerCallbacks();
    app_ble_pServer->setCallbacks(app_ble_pServerCallbacks);

    app_ble_pUartService = app_ble_pServer->createService(SERVICE_UART_UUID);
    app_ble_pUartRxCharacteristic = app_ble_pUartService->createCharacteristic(CHARACTERISTIC_UART_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    app_ble_pUartTxCharacteristic = app_ble_pUartService->createCharacteristic(CHARACTERISTIC_UART_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    app_ble_pUartCallbacks = new app_ble_UartCallbacks;
    app_ble_pUartBLE2902 = new BLE2902();
    app_ble_pUartTxCharacteristic->addDescriptor(app_ble_pUartBLE2902);
    app_ble_pUartRxCharacteristic->setCallbacks(app_ble_pUartCallbacks);

    app_ble_pUartService->start();
    app_ble_pAdvertising = BLEDevice::getAdvertising();
    app_ble_pAdvertising->addServiceUUID(app_ble_pUartService->getUUID());
    app_ble_pAdvertising->setScanResponse(true);
    app_ble_pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    app_ble_pAdvertising->setMinPreferred(0x12);
    app_ble_pAdvertising->start();
    app_ble_txCpm = true;
    initialized = true;
    serviceRequest(SERVICE_DOSIMETRY);
    retVal = true;
  }

  if ((serviceFlags & (1 << SERVICE_DOSIMETRY)) != 0)
  {
    if (app_ble_txCpm) sprintf(printBuf, "%.2f\n", serviceCpm);
    else  sprintf(printBuf, "%.2f\n", serviceUsv);
    app_ble_pUartTxCharacteristic->setValue(printBuf);
    app_ble_pUartTxCharacteristic->notify();
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    app_ble_txCpm = !app_ble_txCpm;
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    app_ble_pAdvertising->stop();
    app_ble_pUartService->stop();
    app_ble_pServer->removeService(app_ble_pUartService);

    delete app_ble_pUartCallbacks;
    delete app_ble_pUartRxCharacteristic;
    delete app_ble_pUartTxCharacteristic;
    delete app_ble_pUartBLE2902;
    delete app_ble_pUartService;

    delete app_ble_pServer;
    delete app_ble_pServerCallbacks;

    app_ble_pUartCallbacks = NULL;
    app_ble_pUartRxCharacteristic = NULL;
    app_ble_pUartTxCharacteristic = NULL;
    app_ble_pUartService = NULL;
    app_ble_pUartBLE2902 = NULL;

    app_ble_pServer = NULL;
    app_ble_pServerCallbacks = NULL;
    BLEDevice::deinit(false);

    initialized = false;
    serviceRelease(SERVICE_DOSIMETRY);
    runningApp = runningApp->parent;
    retVal = true;
  }

  return retVal;
}

int32_t app_ble_maySleepFor()
{
  // Do not go to sleep while BLE is running
  return 0;
}


void app_ble_draw()
{
  u8g2.setFont(u8g2_font_ncenB08_tr);
  if (app_ble_deviceConnected)
  {
    u8g2.drawStr(0,10,"BLE Connected");
    if (app_ble_txCpm)
    {
      u8g2.drawStr(0,20,"Transmitting CPM");
      u8g2.drawStr(45, 30,"uSv/h");
    }
    else
    {
      u8g2.drawStr(0,20,"Transmitting uSv/h");
      u8g2.drawStr(48, 30,"CPM");
    }
  }
  else
  {
    u8g2.drawStr(0,10,"BLE Advertising");
  }
}
