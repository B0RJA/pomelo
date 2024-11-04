#include "esp_sleep.h"

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp32-hal-ledc.h"

#include "soc/soc.h"
#include "esp_intr_alloc.h"

#include "hal/pmu_types.h"
#include "soc/pmu_icg_mapping.h"
#include "soc/pmu_struct.h"

#include "hal/uart_ll.h"
#include "soc/interrupts.h"

#include <ESP32Time.h>
#include <Preferences.h>

#include <ArduinoJson.h>

#include <Arduino.h>
#include <U8g2lib.h>

#include <EasyButton.h>

#include "1euroFilter.h"

#include <WiFi.h>
#include <ESPmDNS.h>


OneEuroFilter *cpm_euroFilter;
OneEuroFilter *usv_euroFilter;

Preferences preferences;
String wifi_ssid, wifi_pass;

U8G2_ST7565_NHD_C12832_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 10, /* data=*/ 0, /* cs=*/ 11, /* dc=*/ 1, /* reset=*/ 22);


// Pin definitions
#define BUT1_A          6
#define BUT2_B          9
#define BUT3_C          7

#define EN_PERI         3
#define MEAS            8
#define HBAT            2
#define BACKLIGHT       15
#define BUZZER          23

#define HIST_LEN           60


EasyButton buttonA(BUT1_A, 35, false, false);
EasyButton buttonB(BUT2_B, 35, false, true);
EasyButton buttonC(BUT3_C, 35, false, false);


ESP32Time rtc;

volatile uint32_t tCanSleep;

float bufHistoryCpm[HIST_LEN + 1];
float bufHistoryUsv[HIST_LEN + 1];

float serviceCpm, serviceUsv, serviceCpmRaw, serviceUsvRaw;
bool serviceUartPulse;
bool serviceRunning;
bool serviceSkipMeasurement;


uint32_t tNow, tLightOff, tService, tServiceWait;
char buf[24576];
char printBuf[80];

uint32_t serviceHist[1024];
uint32_t serviceHistCounts;
uint32_t serviceHistTime;
float serviceHistEcal[3];

float serviceCoreTemp;
bool serviceCoreRunning;
char serviceCoreSN[64];

bool startDaq;

#define BUTTON_1                0
#define BUTTON_2                1
#define BUTTON_3                2
#define BUTTON_1_LONG           3
#define BUTTON_2_LONG           4
#define BUTTON_3_LONG           5

uint8_t pressedButtonFlags;


RTC_DATA_ATTR bool backlightEnabled;
RTC_DATA_ATTR int8_t backlightSeconds;
RTC_DATA_ATTR uint8_t backlightIntensity;


#define UART_BUFFER_LEN 16384
volatile uint16_t rxBufHead, rxBufTail;
volatile uint8_t rxBuf[UART_BUFFER_LEN];

hw_timer_t *Timer0 = NULL;
volatile uint16_t beep_ms;

#define SERVICE_CONFIG          0
#define SERVICE_SYSTEM          1
#define SERVICE_DOSIMETRY       2
#define SERVICE_SPECTROSCOPY    3
#define SERVICE_DATASTREAM      4
#define SERVICE_TICK            5

volatile uint8_t serviceList[8];
uint8_t serviceUpdateFlags;


bool wifi_connected, wifi_AP, wifi_reconnecting, wifi_mdns;
String wifi_mdns_name;
IPAddress wifi_IP;
uint8_t wifi_MAC[6];
uint32_t wifi_tReconnect;



// Fix for [E][AsyncTCP.cpp:1486] begin(): bind error: -8
// ---------------------------------------------------------------------------------------------------
// https://github.com/esp8266/Arduino/tree/master/doc/faq#how-to-clear-tcp-pcbs-in-time-wait-state-
struct tcp_pcb;
extern struct tcp_pcb* tcp_tw_pcbs;
extern "C" void tcp_abort (struct tcp_pcb* pcb);

void tcpCleanup (void) {
  while (tcp_tw_pcbs)
    tcp_abort(tcp_tw_pcbs);
}
// ---------------------------------------------------------------------------------------------------


typedef struct zestApp
{
  struct zestApp *parent;
  char name[64];
  char icon[16];
  bool (*updateUI)(uint8_t buttonFlags);         // Returns false if app quit, so we need to refresh UI with next app
  bool (*updateService)(uint8_t serviceFlags);   // Returns false if it can't handle more data. Only valid for SERVICE_DATASTREAM
  int32_t (*maySleepFor)();
} zestApp_t;


zestApp_t *runningApp;


void noSleepUntil(uint32_t ms)
{
  if (tCanSleep < ms) tCanSleep = ms;
}

void onButtonAPressed()
{
  pressedButtonFlags |= (1 << BUTTON_1);
}

void onButtonBPressed()
{
  pressedButtonFlags |= (1 << BUTTON_2);
}

void onButtonCPressed()
{
  pressedButtonFlags |= (1 << BUTTON_3);
}

void onButtonALongPressed()
{
  pressedButtonFlags |= (1 << BUTTON_1_LONG);
}

void onButtonBLongPressed()
{
  pressedButtonFlags |= (1 << BUTTON_2_LONG);
}

void onButtonCLongPressed()
{
  pressedButtonFlags |= (1 << BUTTON_3_LONG);
}

// Check if TX fifo has space!
void coreCommand(char *cmd)
{
  static uint8_t i, n;
  while (*cmd != 0)
  {
    while (uart_ll_get_txfifo_len(&UART1) == 0) ;
    uart_ll_write_txfifo(&UART1, (const uint8_t*)cmd, 1);
    n = 1;
    if (*cmd == '\n') n = 6;
    
    // This might take a while, so keep checking buttons for activity
    for (i = 0; i < n; i++)
    {
      if (buttonA.read()) noSleepUntil(tNow + 1000);
      if (buttonB.read()) noSleepUntil(tNow + 1000);
      if (buttonC.read()) noSleepUntil(tNow + 1000);
      delay(10);
    }
    cmd++;
  }
  noSleepUntil(millis() + 50);
}

static void IRAM_ATTR timer0_ISR()
{
  ledcWrite(BUZZER, 0);
  timerStop(Timer0);
}

void zestBeep(uint16_t ms)
{
  ledcWrite(BUZZER, 128);
  timerStop(Timer0);
  timerWrite(Timer0, 0);
  timerStart(Timer0);
  timerAlarm(Timer0, ms*1000, false, 0);
  noSleepUntil(tNow + ms + 10);
}


static void IRAM_ATTR coreUartIsr(void *arg)
{
  static uint8_t data;

  while (uart_ll_get_rxfifo_len(&UART1) != 0)
  {
    uart_ll_read_rxfifo(&UART1, &data, 1);
    if (serviceList[SERVICE_DATASTREAM] != 0)
    {
      // Special case if app wants all data
      rxBuf[rxBufHead] = data;
      rxBufHead = (rxBufHead + 1) % UART_BUFFER_LEN;
    }
    else
    {
      if (data == 0xAA)
      {
        zestBeep(beep_ms);
      }
      else if ((data < 128) && (rxBufHead != rxBufTail))
      {
        rxBuf[rxBufHead] = data;
        rxBufHead = (rxBufHead + 1) % UART_BUFFER_LEN;
      }
    }
  }
  uart_ll_clr_intsts_mask(&UART1, UART_INTR_RXFIFO_FULL);
}

// Remove dependency on uart driver. Ugh, not trivial
void serialConfig()
{
  const uart_config_t uart_config = {
      .baud_rate = 921600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, 5, 4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  esp_intr_alloc(ETS_UART1_INTR_SOURCE, 0, coreUartIsr, NULL, NULL);

  rxBufHead = 1;
  rxBufTail = 0;
  uart_ll_set_rxfifo_full_thr(&UART1, 1);
  uart_ll_ena_intr_mask(&UART1, UART_INTR_RXFIFO_FULL);
}

bool serialAvailable()
{
  if ((rxBufTail + 1) % UART_BUFFER_LEN != rxBufHead) return true;
  else return false;
}

char serialRead()
{
  rxBufTail = (rxBufTail + 1) % UART_BUFFER_LEN;
  return rxBuf[rxBufTail];
}



void zest_lightsleep(int64_t time)
{
  static esp_sleep_wakeup_cause_t wakeup_reason;

  // This has a lot of overhead, so let's only comply if requested sleep is at least 20 ms
  // and we sleep for 10 ms less just to allow for other housekeeping tasks to occur on time.
  // Also we're sometimes called with negative time so make sure we ignore those.
  if (time < 20) return;
  time -= 10;

  // Set which pins we need to hold, starting with LCD
  gpio_hold_en(GPIO_NUM_0);
  gpio_hold_en(GPIO_NUM_1);
  gpio_hold_en(GPIO_NUM_10);
  gpio_hold_en(GPIO_NUM_11);
  gpio_hold_en(GPIO_NUM_22);
  gpio_hold_en(GPIO_NUM_5); // CORE_RX
  gpio_hold_en(GPIO_NUM_8); // MEAS
  gpio_hold_en(GPIO_NUM_3); // EN_PERI
  //gpio_hold_en(GPIO_NUM_15); // Backlight PWM keeps running in light sleep
  gpio_hold_en(GPIO_NUM_23); // Buzzer
  gpio_hold_en(GPIO_NUM_18);
  gpio_hold_en(GPIO_NUM_19);
  gpio_hold_en(GPIO_NUM_20);
  gpio_hold_en(GPIO_NUM_21);
  gpio_hold_en(GPIO_NUM_2);   // HBAT

  // Light sleep
  esp_sleep_enable_timer_wakeup(time*1000);

  gpio_wakeup_enable(GPIO_NUM_6, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_7, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  gpio_sleep_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
  gpio_sleep_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
  uart_set_wakeup_threshold(UART_NUM_1, 3);
  esp_sleep_enable_uart_wakeup(UART_NUM_1);

  esp_light_sleep_start();
  wakeup_reason = esp_sleep_get_wakeup_cause();

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // Unhold pins
  gpio_hold_dis(GPIO_NUM_0);
  gpio_hold_dis(GPIO_NUM_1);
  gpio_hold_dis(GPIO_NUM_10);
  gpio_hold_dis(GPIO_NUM_11);
  gpio_hold_dis(GPIO_NUM_22);
  gpio_hold_dis(GPIO_NUM_5); // CORE_RX
  gpio_hold_dis(GPIO_NUM_8); // MEAS
  gpio_hold_dis(GPIO_NUM_3); // EN_PERI
  //gpio_hold_dis(GPIO_NUM_15); // Backlight
  gpio_hold_dis(GPIO_NUM_23); // Buzzer
  gpio_hold_dis(GPIO_NUM_18);
  gpio_hold_dis(GPIO_NUM_19);
  gpio_hold_dis(GPIO_NUM_20);
  gpio_hold_dis(GPIO_NUM_21);
  gpio_hold_dis(GPIO_NUM_2);   // HBAT

  tNow = millis();

  if (buttonA.read()) noSleepUntil(tNow + 1000);
  if (buttonB.read()) noSleepUntil(tNow + 1000);
  if (buttonC.read()) noSleepUntil(tNow + 1000);

  while (serialAvailable()) serialRead();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_UART)
  {
    zestBeep(beep_ms);
  }
}

void zest_deepsleep()
{
  u8g2.setPowerSave(1);
  delay(100);

  // Set all LCD pins as outputs, low
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(22, LOW);

  // Set all SD card pins as outputs, low
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
  digitalWrite(18, LOW);
  digitalWrite(19, LOW);

  digitalWrite(EN_PERI, HIGH);
  backlight(1); // Fade out
  delay(400);

  rtc_gpio_hold_en(GPIO_NUM_0);
  rtc_gpio_hold_en(GPIO_NUM_1);
  rtc_gpio_hold_en(GPIO_NUM_10);
  rtc_gpio_hold_en(GPIO_NUM_11);
  rtc_gpio_hold_en(GPIO_NUM_22);
  rtc_gpio_hold_en(GPIO_NUM_5); // CORE_RX
  rtc_gpio_hold_en(GPIO_NUM_8); // MEAS
  rtc_gpio_hold_en(GPIO_NUM_3); // EN_PERI
  rtc_gpio_hold_en(GPIO_NUM_15); // Backlight
  rtc_gpio_hold_en(GPIO_NUM_23); // Buzzer
  rtc_gpio_hold_en(GPIO_NUM_18);
  rtc_gpio_hold_en(GPIO_NUM_19);
  rtc_gpio_hold_en(GPIO_NUM_20);
  rtc_gpio_hold_en(GPIO_NUM_21);
  rtc_gpio_hold_en(GPIO_NUM_2);   // HBAT

  esp_sleep_enable_ext1_wakeup_io(0xC0, ESP_EXT1_WAKEUP_ANY_HIGH);
  rtc_gpio_pulldown_dis(GPIO_NUM_6);
  rtc_gpio_pullup_dis(GPIO_NUM_6);
  rtc_gpio_pulldown_dis(GPIO_NUM_7);
  rtc_gpio_pullup_dis(GPIO_NUM_7);
  esp_deep_sleep_start();
  // Chip dead at this point. Will wake up through reset
}


void updateHistory(float newValue, float *history)
{
  uint16_t i;
  history[HIST_LEN] = newValue; // array is HIST_LEN + 1 sized
  for (i = 0; i < HIST_LEN; i++)
  {
    history[i] = history[i + 1];
  }
}

void drawHistory(float *history)
{
  float historyMax;
  uint16_t i;
  uint8_t s0, s1;

  historyMax = 0;
  for (i = 0; i < HIST_LEN; i++)
  {
    if (history[i] > historyMax) historyMax = history[i];
  }

  for (i = 0; i < HIST_LEN - 1; i++)
  {
    s0 = 31.0*history[i]/historyMax;
    s1 = 31.0*history[i+1]/historyMax;
    u8g2.drawLine(i, 31 - s0, i + 1, 31 - s1);
  }

  // Draw cursor

  i = HIST_LEN;
  s0 = 31.0*history[HIST_LEN]/historyMax;
  u8g2.drawTriangle(i, 31 - s0, i + 3, 31 - s0 - 3, i + 3, 31 - s0 + 3);

  u8g2.drawLine(64, 0, 64, 32);
}


zestApp_t app_menu = {NULL, "a", "a", &app_menu_updateUI, &app_menu_updateService, NULL};

zestApp_t mainMenuList[] = 
{
  {&app_menu, "Power",    "\xEB",   &app_power_updateUI,    &app_power_updateService,     NULL},
  {&app_menu, "CPM",      "\x8D",   &app_cpm_updateUI,      &app_cpm_updateService,       NULL},
  {&app_menu, "uSv/h",    "\x64",   &app_usv_updateUI,      &app_usv_updateService,       NULL},
  {&app_menu, "Spectrum", "\x58",   &app_sp_updateUI,       &app_sp_updateService,        NULL},
  {&app_menu, "HTTP",     "\xF7",   &app_http_updateUI,     &app_http_updateService,      &app_http_maySleepFor},
  {&app_menu, "TCP",      "\xF7",   &app_tcp_updateUI,      &app_tcp_updateService,       &app_tcp_maySleepFor},
  {&app_menu, "Endpoint", "\x80",   &app_endpoint_updateUI, &app_endpoint_updateService,  NULL},
  {&app_menu, "BLE",      "\x5E",   &app_ble_updateUI,      &app_ble_updateService,       &app_ble_maySleepFor},
  {&app_menu, "SD Log",   "\x80",   &app_log_updateUI,      &app_log_updateService,       NULL},
  {&app_menu, "Settings", "\x81",   &app_settings_updateUI, &app_settings_updateService,  NULL},
  {&app_menu, "Info",     "\xBC",   &app_info_updateUI,     &app_info_updateService,      NULL},
};


void serviceRequest(uint8_t service)
{
  uint8_t i;

  // If we just start the dosimetry
  // service, make sure we start fresh
  if ( (service == SERVICE_DOSIMETRY) && (serviceList[service] == 0) )
  {
    serviceSkipMeasurement = true;
    cpm_euroFilter = new OneEuroFilter(1.0, 0.002670, 0.0000153);
    usv_euroFilter = new OneEuroFilter(1.0, 0.010515, 0.042970);
    serviceCpm = 0;
    serviceCpmRaw = 0;
    serviceUsv = 0;
    serviceUsvRaw = 0;
    for (i = 0; i < HIST_LEN; i++)
    {
      bufHistoryCpm[i] = 0;
      bufHistoryUsv[i] = 0;
    }
  }

  serviceList[service]++;
}

void serviceRelease(uint8_t service)
{
  if (serviceList[service] > 0)
  {
    serviceList[service]--;

    // If we need to stop the dosimetry
    // service, make sure we clean up
    if ( (service == SERVICE_DOSIMETRY) && (serviceList[service] == 0) )
    {
      delete cpm_euroFilter;
      delete usv_euroFilter;
    }
  }
}

// Returns if a service has been triggered and is now running
bool serviceCall(bool restart)
{
  static uint8_t serviceNum = 0;
  uint8_t i;

  if (restart) serviceNum = 0;

  // First 4 services represent Core commands
  while ((serviceList[serviceNum] == 0) && (serviceNum < 4))
  {
    serviceNum++;
  }

  if (serviceNum < 4)
  {
    switch (serviceNum)
    {
      case SERVICE_CONFIG:
        coreCommand("c");
        break;
      case SERVICE_SYSTEM:
        coreCommand("s");
        break;
      case SERVICE_DOSIMETRY:
        coreCommand("m");
        break;
      case SERVICE_SPECTROSCOPY:
        coreCommand("h");
        break;
    }
    serviceNum++;
    return true;
  }
  else
  {
    return false;
  }
}

void app_menuScreen(char *name, char *icon)
{
  uint8_t i;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso16_tf);
  i = u8g2.getStrWidth(name);
  u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, name);
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawStr(64 - (16 + 4 + i)/2, 24, icon);
  u8g2.drawTriangle(0, 15, 10, 0, 10, 30);
  u8g2.drawTriangle(127, 15, 117, 0, 117, 30);
  u8g2.sendBuffer();
}

bool app_menu_updateUI(uint8_t buttonFlags)
{
  static int8_t menuSelection = 0;
  static int8_t numApps = sizeof(mainMenuList) / sizeof(zestApp_t);

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    // go backwards
    menuSelection--;
    if (menuSelection < 0) menuSelection = numApps - 1;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    // go forward
    menuSelection++;
    if (menuSelection >= numApps) menuSelection = 0;
  }

  app_menuScreen(mainMenuList[menuSelection].name, mainMenuList[menuSelection].icon);

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    runningApp = &mainMenuList[menuSelection];
    return false;
  }

  return true;
}

bool app_menu_updateService(uint8_t serviceFlags)
{
  return true;
}

void backlight(uint8_t command)
{
  static bool lightOn = false;
  if (command == 3)                     // Go immediately to level
  {
    ledcWrite(BACKLIGHT, backlightIntensity);
    lightOn = true;
  }
  if (!lightOn && (command == 2))       // Switch on
  {
    ledcWrite(BACKLIGHT, backlightIntensity);
    lightOn = true;
  }
  else if (lightOn && (command == 1))   // Fade out
  {
    ledcFade(BACKLIGHT, backlightIntensity, 0, 300);
    lightOn = false;
  }
  else if (command == 0)                // Off immediately
  {
    ledcWrite(BACKLIGHT, 0);
    lightOn = false;
  }

  if (lightOn) tLightOff = millis() + backlightSeconds * 1000;
}


void setup()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  uint16_t i;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  if ((wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) || (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER))
  {
    // Start by making sure the pins are properly accessible
    rtc_gpio_hold_dis(GPIO_NUM_0);
    rtc_gpio_hold_dis(GPIO_NUM_1);
    rtc_gpio_hold_dis(GPIO_NUM_10);
    rtc_gpio_hold_dis(GPIO_NUM_11);
    rtc_gpio_hold_dis(GPIO_NUM_22);
    rtc_gpio_hold_dis(GPIO_NUM_5); // CORE_RX
    rtc_gpio_hold_dis(GPIO_NUM_8); // MEAS
    rtc_gpio_hold_dis(GPIO_NUM_3); // EN_PERI
    rtc_gpio_hold_dis(GPIO_NUM_15); // Backlight
    rtc_gpio_hold_dis(GPIO_NUM_23); // Buzzer
    rtc_gpio_hold_dis(GPIO_NUM_18);
    rtc_gpio_hold_dis(GPIO_NUM_19);
    rtc_gpio_hold_dis(GPIO_NUM_20);
    rtc_gpio_hold_dis(GPIO_NUM_21);
    rtc_gpio_hold_dis(GPIO_NUM_2);   // HBAT
  }

  // Can this be made to work without rebooting ESP32 on connect / disconnect?
  //Serial.begin(115200);   // USB CDC

  preferences.begin("pomeloZest", false);
  wifi_ssid = preferences.getString("ssid", ""); 
  wifi_pass = preferences.getString("password", "");
  preferences.end();

  pinMode(BUT1_A, INPUT);
  pinMode(BUT2_B, INPUT);
  pinMode(BUT3_C, INPUT);

  buttonA.begin();
  buttonB.begin();
  buttonC.begin();

  buttonA.onPressed(onButtonAPressed);
  buttonB.onPressed(onButtonBPressed);
  buttonC.onPressed(onButtonCPressed);

  buttonA.onPressedFor(750, onButtonALongPressed);
  buttonB.onPressedFor(750, onButtonBLongPressed);
  buttonC.onPressedFor(750, onButtonCLongPressed);

  pressedButtonFlags = 0;

  pinMode(EN_PERI, OUTPUT);
  pinMode(MEAS, OUTPUT);
  pinMode(BACKLIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(EN_PERI, LOW);
  digitalWrite(MEAS, HIGH);
  digitalWrite(BACKLIGHT, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(2, LOW);

  u8g2.begin();

  // Espressif hackery to make PWM work in light sleep
  esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_ON);
  uint32_t val = PMU.hp_sys[PMU_MODE_HP_SLEEP].icg_func;
  PMU.hp_sys[PMU_MODE_HP_SLEEP].icg_func = (val | BIT(PMU_ICG_FUNC_ENA_LEDC) | BIT(PMU_ICG_FUNC_ENA_IOMUX));
  ledcSetClockSource(LEDC_USE_RC_FAST_CLK);

  ledcAttach(BACKLIGHT, 2000, 8);
  ledcWrite(BACKLIGHT, 0);
  gpio_sleep_sel_dis(GPIO_NUM_15);

  ledcAttach(BUZZER, 2000, 8);
  ledcWrite(BUZZER, 0);
  beep_ms = 2;
  Timer0 = timerBegin(1000000);
  timerAttachInterrupt(Timer0, timer0_ISR);
  timerStop(Timer0);

  tNow = millis();
  tCanSleep = tNow;
  tLightOff = tNow;

  for (i = 0; i < 1024; i++)
  {
    serviceHist[i] = 0;
  }

  for (i = 0; i < HIST_LEN; i++)
  {
    bufHistoryCpm[i] = 0;
    bufHistoryUsv[i] = 0;
  }

  serviceHistCounts = 0;
  serviceHistTime = 0;
  serviceCoreTemp = -273;

  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED)
  {
    // Hard reset
    backlightEnabled = true;
    backlightSeconds = 5;
    backlightIntensity = 50;
  }

  for (i = 0; i++; i < 8) serviceList[i] = 0;
  serviceRunning = false;
  serviceSkipMeasurement = false;
  serviceUpdateFlags = 0;

  if (backlightEnabled) backlight(2); // Switch on

  runningApp = &app_menu;
  runningApp->updateUI(0);

  serialConfig();
  while (serialAvailable()) serialRead();

  // Make pulse char 0xAA (170 decimal),
  // such that it has several transitions
  coreCommand("l170\n");

  // If DAQ is stopped it will be restarted. Otherwise
  // it'll let it continue undisturbed
  coreCommand("s");
  startDaq = true;


  wifi_connected = false;
  wifi_AP = false;
  wifi_reconnecting = false;
  wifi_tReconnect = 0;
  wifi_mdns = false;
}



void loop()
{
  static uint16_t bufIdx = 0;
  static uint8_t rxChar;
  static int32_t maySleep, maySleepApp;

  tNow = millis();

  if (buttonA.read()) noSleepUntil(tNow + 1000);
  if (buttonB.read()) noSleepUntil(tNow + 1000);
  if (buttonC.read()) noSleepUntil(tNow + 1000);

  if (tNow >= tCanSleep)
  {
    maySleep = tService - tNow;
    maySleepApp = maySleep; // just to have a sensible value in case app does not care

    if (runningApp->maySleepFor != NULL) maySleepApp = runningApp->maySleepFor();
    if (maySleepApp < maySleep) maySleep = maySleepApp;

    if (serviceRunning) maySleepApp = tServiceWait - tNow;
    if (maySleepApp < maySleep) maySleep = maySleepApp;

    if (backlightEnabled && (tLightOff > tNow))
    {
      maySleepApp = (int64_t)tLightOff - (int64_t)tNow;
      if (maySleepApp < maySleep) maySleep = maySleepApp;
    }

    zest_lightsleep(maySleep);
  }

  if (pressedButtonFlags != 0)
  {
    if (backlightEnabled) backlight(2); // Switch on
  }

  if (tNow >= tService)
  {
    tService += 1000;
    serviceRunning = serviceCall(true);
    tServiceWait = tNow + 150;
    if (serviceList[SERVICE_TICK] != 0) serviceUpdateFlags |= (1 << SERVICE_TICK);
  }
  
  if (serviceRunning)
  {
    if (tNow >= tServiceWait)
    {
      serviceRunning = serviceCall(false);
      tServiceWait = tNow + 150;
    }
  }

  if (serviceUpdateFlags != 0)
  {
    runningApp->updateService(serviceUpdateFlags);
    serviceUpdateFlags = 0;
  }

  if (pressedButtonFlags != 0)
  {
    if (!runningApp->updateUI(pressedButtonFlags))
    {
      // Previous app quit. Run update on new
      // app to update display
      runningApp->updateUI(0);
    }
    pressedButtonFlags = 0;
  }

  if (serviceList[SERVICE_DATASTREAM] != 0)
  {
    // Just keep application running as this
    // needs to also monitor incoming connections
    runningApp->updateUI(0);
  }

  if ((tNow >= tLightOff) && (backlightSeconds != 0))
  {
    backlight(1); // Fade out
  }

  if (serialAvailable())
  {
    noSleepUntil(tNow + 50);
    if (serviceList[SERVICE_DATASTREAM] != 0)
    {
      while (serialAvailable())
      {
        if (!runningApp->updateService(1 << SERVICE_DATASTREAM))
          break;
      }
    }
    else
    {
      while (serialAvailable())
      {
        rxChar = serialRead();
        if (rxChar == '\n')
        {
          buf[bufIdx] = 0;
          parseBuffer(buf);
          bufIdx = 0;
        }
        else
        {
          buf[bufIdx] = rxChar;
          if (bufIdx < sizeof(buf))
          {
            bufIdx++;
          }
          else
          {
            bufIdx = 0;
          }
        }
      }
    }
  }
}


void parseBuffer(char* payload)
{
  DynamicJsonDocument doc(49152);
  DeserializationError error = deserializeJson(doc, payload);

  if (error)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "JSON Error");
    u8g2.drawStr(0, 20, error.c_str());
    u8g2.drawStr(0, 30, buf);
    u8g2.sendBuffer();
    delay(100);
    return;
  }

  // Next service (if necessary) can be called
  // on the next loop iteration
  tServiceWait = tNow;

  // Throw in a button check just for good measure. Application
  // parseData might take a while to update the screen
  if (buttonA.read()) noSleepUntil(tNow + 1000);
  if (buttonB.read()) noSleepUntil(tNow + 1000);
  if (buttonC.read()) noSleepUntil(tNow + 1000);

  if (doc["type"] == "dosimetry") parseDataDosimetry(&doc);
  if (doc["type"] == "spectrum") parseDataSpectrum(&doc);
  if (doc["type"] == "config") parseDataConfig(&doc);
  if (doc["type"] == "system") parseDataSystem(&doc);
}


void parseDataDosimetry(DynamicJsonDocument *doc)
{
  float cpm, usv;
  uint16_t i;

  cpm = (*doc)["payload"]["cpm"].as<float>();
  usv = (*doc)["payload"]["uSv/h"].as<float>();

  if (serviceSkipMeasurement)
  {
    serviceSkipMeasurement = false;
  }
  else
  {
    serviceCpmRaw = cpm;
    serviceCpm = cpm_euroFilter->filter(cpm);
    serviceUsvRaw = usv;
    serviceUsv = usv_euroFilter->filter(usv);
    bufHistoryCpm[HIST_LEN] = serviceCpm;
    bufHistoryUsv[HIST_LEN] = serviceUsv;
    for (i = 0; i < HIST_LEN; i++)
    {
      bufHistoryCpm[i] = bufHistoryCpm[i + 1];
      bufHistoryUsv[i] = bufHistoryUsv[i + 1];
    }
    serviceUpdateFlags |= (1 << SERVICE_DOSIMETRY);
  }
}

void parseDataConfig(DynamicJsonDocument *doc)
{
  uint32_t uartPulse;
  uartPulse = (*doc)["payload"]["outputs"]["uart"]["fastPulse"].as<int>();
  if (uartPulse != 0) serviceUartPulse = true;
  else serviceUartPulse = false;
  serviceUpdateFlags |= (1 << SERVICE_CONFIG);
}

void parseDataSystem(DynamicJsonDocument *doc)
{
  serviceCoreTemp = (*doc)["payload"]["temperature"].as<float>();
  serviceCoreRunning = (*doc)["payload"]["running"].as<int>();
  strcpy(serviceCoreSN, (*doc)["payload"]["sn"].as<String>().c_str());
  if (startDaq)
  {
    startDaq = false;
    if (serviceCoreRunning == 0) coreCommand("x");
  }
  serviceUpdateFlags |= (1 << SERVICE_SYSTEM);
}

void parseDataSpectrum(DynamicJsonDocument *doc)
{
  uint16_t i;

  for (i = 0; i < 1024; i++)
  {
    serviceHist[i] = (*doc)["payload"]["data"][i].as<unsigned long long>();
  }

  serviceHistCounts = (*doc)["payload"]["count"].as<unsigned long long>();
  serviceHistTime = (*doc)["payload"]["time"].as<unsigned long long>();
  serviceCoreTemp = (*doc)["payload"]["temperature"].as<float>();
  serviceHistEcal[0] = (*doc)["payload"]["ecal"][0].as<float>();
  serviceHistEcal[1] = (*doc)["payload"]["ecal"][1].as<float>();
  serviceHistEcal[2] = (*doc)["payload"]["ecal"][2].as<float>();

  serviceCoreTemp = (*doc)["payload"]["temperature"].as<float>();
  serviceCoreRunning = (*doc)["payload"]["running"].as<int>();
  strcpy(serviceCoreSN, (*doc)["payload"]["sn"].as<String>().c_str());
  
  serviceUpdateFlags |= (1 << SERVICE_SYSTEM) | (1 << SERVICE_SPECTROSCOPY);
}


void drawSpectrum(bool log)
{
  uint16_t i;
  float spectrum[128];
  float spectrumMax;
  uint8_t s;

  for (i = 0; i < 128; i++)
  {
    spectrum[i] = 0;
  }
  
  for (i = 0; i < 1024; i++)
  {
    spectrum[i/8] += serviceHist[i];
  }

  if (log)
  {
    for (i = 0; i < 128; i++)
    {
      if (spectrum[i] >= 1) spectrum[i] = log10(spectrum[i]);
      else spectrum[i] = 0;
    }
  }

  spectrumMax = 0;
  for (i = 0; i < 128; i++)
  {
    if (spectrum[i] > spectrumMax) spectrumMax = spectrum[i];
  }

  u8g2.clearBuffer();
  for (i = 0; i < 128; i++)
  {
    s = 31.0*spectrum[i]/spectrumMax;
    if (s != 0) u8g2.drawLine(i, 31, i, 31 - s);
  }
  u8g2.setFont(u8g2_font_t0_11b_tr);
  if (log)
  {
    u8g2.drawStr(0, 10, "L");
    u8g2.drawStr(0, 20, "o");
    u8g2.drawStr(0, 30, "g");
  }
  else
  {
    u8g2.drawStr(0, 10, "L");
    u8g2.drawStr(0, 20, "i");
    u8g2.drawStr(0, 30, "n");
  }
  sprintf(printBuf, "%d", serviceHistCounts);
  u8g2.drawStr(80, 10, printBuf);
  u8g2.sendBuffer();
}


void wifiConnectAP()
{
  if (!wifi_connected)
  {
    WiFi.mode(WIFI_AP);
    WiFi.enableAP(true);
    Network.macAddress(wifi_MAC);
    sprintf(printBuf, "P-%02X%02X", wifi_MAC[4], wifi_MAC[5]);
    WiFi.softAP(printBuf, "pomelopw");
    wifi_IP = WiFi.softAPIP();
    wifi_connected = true;
    wifi_AP = true;
  }
}

void wifiConnectSTA()
{
  if (!wifi_connected)
  {
    if (wifi_ssid != "")
    {
      WiFi.mode(WIFI_STA);
      WiFi.enableSTA(true);
      WiFi.begin(wifi_ssid, wifi_pass);
      for (int i = 0; i < 10; i++)
      {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(10,10, "Connection attempt");
        sprintf(printBuf, "%d/10", i);
        u8g2.drawStr(55,20, printBuf);
        u8g2.sendBuffer();

        if (WiFi.status() == WL_CONNECTED)
        {
          break;
        }
        delay(300);
      }

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      if (WiFi.status() == WL_CONNECTED)
      {
        wifi_IP = WiFi.localIP();
        wifi_connected = true;
        wifi_AP = false;
        u8g2.drawStr(0,10, "Connected to network");
      }
      else
      {
        WiFi.disconnect(true, false, 0);
        WiFi.mode(WIFI_OFF);
        u8g2.drawStr(0,10, "Connection failed");
      }
      u8g2.sendBuffer();
      delay(500);
    }
  }
}

void wifiDisconnect()
{
  if (wifi_connected)
  {
    if (wifi_AP)
    {
      WiFi.softAPdisconnect(true); 
    }
    else
    {
      WiFi.disconnect(true, false, 0);
    }
    delay(300);
    WiFi.mode(WIFI_OFF);
    wifi_connected = false;
    wifi_AP = false;
  }
}

void wifiCheck()
{
  if (wifi_connected && !wifi_AP)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      wifi_reconnecting = true;
      wifi_tReconnect = 0;
    }
  }

  if (wifi_reconnecting)
  {
    if (wifi_tReconnect == 0)
    {
      if (WiFi.status() != WL_CONNECTED)
      {
        WiFi.disconnect(true, false, 0);
        delay(300);
        WiFi.begin(wifi_ssid, wifi_pass);
        wifi_tReconnect = 5;
      }
      else
      {
        wifi_reconnecting = false;
      }
    }
    else
    {
      wifi_tReconnect--;
    }
  }
}

void wifiStartMdns()
{
  esp_ip4_addr_t addr;

  if (mdns_init())
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(10,10, "mDNS failed");
    u8g2.sendBuffer();
    delay(500);
    wifi_mdns = false;
    return;
  }

  wifi_mdns_name = "pomelo";
  // Check for name conflict
  esp_err_t err = mdns_query_a(wifi_mdns_name.c_str(), 2000, &addr);
  if (err != ESP_ERR_NOT_FOUND)
  {
    // err == ESP_ERR_NOT_FOUND is the only good case. Otherwise we
    // need to set up a more specific name for ourselves to not
    // cause conflicts or confusion
    Network.macAddress(wifi_MAC);
    sprintf(printBuf, "pomelo-%02x%02x", wifi_MAC[4], wifi_MAC[5]);
    wifi_mdns_name = printBuf;
  }

  if (mdns_hostname_set(wifi_mdns_name.c_str()))
  {
    wifi_mdns = false;
    mdns_free();
    return;
  }

  wifi_mdns = true;
}

void wifiStopMdns()
{
  if (wifi_mdns)
  {
    MDNS.end();
    wifi_mdns = false;
  }
}
