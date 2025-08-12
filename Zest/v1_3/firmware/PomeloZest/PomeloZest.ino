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

#include "soc/usb_serial_jtag_struct.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <ESP32Time.h>
#include <Preferences.h>

#include <ArduinoJson.h>

#include <Arduino.h>
#include <U8g2lib.h>

#include <EasyButton.h>

#include "1euroFilter.h"

#include <WiFi.h>
#include <ESPmDNS.h>

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial0);

#if !ARDUINO_USB_CDC_ON_BOOT
HWCDC HWCDCSerial;
#endif

OneEuroFilter *cpm_euroFilter;
OneEuroFilter *usv_euroFilter;

Preferences preferences;
String wifi_ssid, wifi_pass;
String startup_script;
int8_t startup_script_idx;

U8G2_ST7565_NHD_C12832_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 10, /* data=*/ 0, /* cs=*/ 11, /* dc=*/ 1, /* reset=*/ 22);


// Pin definitions
#define BUT1_A          6
#define BUT2_B          7
#define BUT3_C          9

#define EN_PERI         3
#define MEAS            8
#define HBAT            2
#define BACKLIGHT       15
#define BUZZER          23

#define HIST_LEN           60


EasyButton buttonA(BUT1_A, 35, false, true);
EasyButton buttonB(BUT2_B, 35, false, true);
EasyButton buttonC(BUT3_C, 35, false, true);


ESP32Time rtc;

volatile uint32_t tCanSleepWatermark, tCanSleepExactly;
volatile bool buzzerActive;

float bufHistoryCpm[HIST_LEN + 1];
float bufHistoryUsv[HIST_LEN + 1];

float serviceCpm, serviceUsv, serviceCpmRaw, serviceUsvRaw;
bool serviceUartPulse;
bool serviceRunning;
bool serviceSkipMeasurement;

uint32_t tNow, tLightOff, tService, tServiceWait;
char buf[24576];
char printBuf[160];

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
uint32_t buttonLockUntil;

RTC_DATA_ATTR bool backlightEnabled;
RTC_DATA_ATTR bool usbData;
RTC_DATA_ATTR bool lcdFlip;
RTC_DATA_ATTR int8_t backlightSeconds;
RTC_DATA_ATTR uint8_t backlightIntensity;
RTC_DATA_ATTR int8_t mainMenuSelection;


#define UART_BUFFER_LEN 16384
volatile uint16_t rxBufHead, rxBufTail;
volatile uint8_t rxBuf[UART_BUFFER_LEN];

hw_timer_t *Timer0 = NULL;
volatile uint16_t beep_ms;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


#define SERVICE_CONFIG          0
#define SERVICE_SYSTEM          1
#define SERVICE_DOSIMETRY       2
#define SERVICE_SPECTROSCOPY    3
#define SERVICE_TICK            4
#define SERVICE_GPS             5

volatile uint8_t serviceList[8];
uint8_t serviceUpdateFlags;


bool wifi_connected, wifi_AP, wifi_reconnecting, wifi_mdns;
String wifi_mdns_name;
IPAddress wifi_IP;
uint8_t wifi_MAC[6];
uint32_t wifi_tReconnect;


typedef struct zestApp
{
  struct zestApp *parent;
  char name[64];
  char icon[16];
  bool (*run)(uint8_t buttonFlags, uint8_t serviceFlags);   // Returns true if screen should redraw
  void (*draw)(void);                                       // Should not contain LCD clear and sendBuffer
  bool (*stream)(uint8_t data);
  int32_t (*maySleepFor)();
} zestApp_t;


RTC_DATA_ATTR zestApp_t *runningApp;


void noSleepUntilWatermark(uint32_t ms)
{
  if (tCanSleepWatermark < ms) tCanSleepWatermark = ms;
}

// Serial also needs to be shortened when receiving '\n'
void noSleepUntilExactly(uint32_t ms)
{
  tCanSleepExactly = ms;
}

void onButtonAPressed()
{
  if (lcdFlip) pressedButtonFlags |= (1 << BUTTON_3);
  else pressedButtonFlags |= (1 << BUTTON_1);
}

void onButtonBPressed()
{
  pressedButtonFlags |= (1 << BUTTON_2);
}

void onButtonCPressed()
{
  if (lcdFlip) pressedButtonFlags |= (1 << BUTTON_1);
  else pressedButtonFlags |= (1 << BUTTON_3);
}

void onButtonALongPressed()
{
  if (lcdFlip) pressedButtonFlags |= (1 << BUTTON_3_LONG);
  else pressedButtonFlags |= (1 << BUTTON_1_LONG);
}

void onButtonBLongPressed()
{
  pressedButtonFlags |= (1 << BUTTON_2_LONG);
}

void onButtonCLongPressed()
{
  if (lcdFlip) pressedButtonFlags |= (1 << BUTTON_1_LONG);
  else pressedButtonFlags |= (1 << BUTTON_3_LONG);
}

void coreCommand(const String &tx)
{
  const char *cmd = tx.c_str();

  while (*cmd != 0)
  {
    while (uart_ll_get_txfifo_len(&UART1) == 0)
    {
      if (buttonA.read()) noSleepUntilWatermark(millis() + 1000);
      if (buttonB.read()) noSleepUntilWatermark(millis() + 1000);
      if (buttonC.read()) noSleepUntilWatermark(millis() + 1000);
    }
    uart_ll_write_txfifo(&UART1, (const uint8_t*)cmd, 1);
    cmd++;
  }
  noSleepUntilWatermark(millis() + 250);
}


static void IRAM_ATTR timer0_ISR()
{
  portENTER_CRITICAL_ISR(&timerMux);
  ledcWrite(BUZZER, 0);
  timerStop(Timer0);
  buzzerActive = false;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void zestBeep(uint16_t ms)
{
  if (serviceUartPulse)
  {
    portENTER_CRITICAL(&timerMux);
    if (buzzerActive)
    {
      timerStop(Timer0);
    }
    ledcWrite(BUZZER, 128);
    timerWrite(Timer0, 0);
    timerAlarm(Timer0, ms*1000, false, 0);
    timerStart(Timer0);
    buzzerActive = true;
    portEXIT_CRITICAL(&timerMux);
  }
}


static void IRAM_ATTR coreUartIsr(void *arg)
{
  static uint8_t data;

  while (uart_ll_get_rxfifo_len(&UART1) != 0)
  {
    uart_ll_read_rxfifo(&UART1, &data, 1);
    if (runningApp->stream != NULL)
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


void zest_gpioHold()
{
  // Set which pins we need to hold except buttons
  // and GPIO15 which is PWM
  rtc_gpio_hold_en(GPIO_NUM_0); // LCD_SI
  rtc_gpio_hold_en(GPIO_NUM_1); // LCD_A0
  rtc_gpio_hold_en(GPIO_NUM_2); // HBAT
  rtc_gpio_hold_en(GPIO_NUM_3); // EN_PERI
  rtc_gpio_hold_en(GPIO_NUM_5); // CORE_RX

  gpio_hold_en(GPIO_NUM_8);   // MEAS
  gpio_hold_en(GPIO_NUM_10);  // LCD_SCL
  gpio_hold_en(GPIO_NUM_11);  // LCD_CS
  gpio_hold_en(GPIO_NUM_18);  // SD_CS
  gpio_hold_en(GPIO_NUM_19);  // SD_MOSI
  gpio_hold_en(GPIO_NUM_20);  // SD_MISO
  gpio_hold_en(GPIO_NUM_21);  // SD_SCK
  gpio_hold_en(GPIO_NUM_22);  // LCD_RST
  gpio_hold_en(GPIO_NUM_23);  // Buzzer
}

void zest_gpioUnhold()
{
  // Unhold pins
  rtc_gpio_hold_dis(GPIO_NUM_0); // LCD_SI
  rtc_gpio_hold_dis(GPIO_NUM_1); // LCD_A0
  rtc_gpio_hold_dis(GPIO_NUM_2); // HBAT
  rtc_gpio_hold_dis(GPIO_NUM_3); // EN_PERI
  rtc_gpio_hold_dis(GPIO_NUM_5); // CORE_RX

  gpio_hold_dis(GPIO_NUM_8);   // MEAS
  gpio_hold_dis(GPIO_NUM_10);  // LCD_SCL
  gpio_hold_dis(GPIO_NUM_11);  // LCD_CS
  gpio_hold_dis(GPIO_NUM_18);  // SD_CS
  gpio_hold_dis(GPIO_NUM_19);  // SD_MOSI
  gpio_hold_dis(GPIO_NUM_20);  // SD_MISO
  gpio_hold_dis(GPIO_NUM_21);  // SD_SCK
  gpio_hold_dis(GPIO_NUM_22);  // LCD_RST
  gpio_hold_dis(GPIO_NUM_23);  // Buzzer
}


void zest_lightsleep(int64_t time)
{
  static esp_sleep_wakeup_cause_t wakeup_reason;

  // If buzzer is beeping don't
  if (buzzerActive) return;

  // This has a lot of overhead, so let's only comply if requested sleep is at least 20 ms
  // and we sleep for 10 ms less just to allow for other housekeeping tasks to occur on time.
  // Also we're sometimes called with negative time so make sure we ignore those.
  if (time < 20) return;
  time -= 10;

  zest_gpioHold();

  esp_sleep_enable_timer_wakeup(time*1000);

  gpio_wakeup_enable(GPIO_NUM_6, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_7, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  gpio_sleep_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
  gpio_sleep_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
  uart_set_wakeup_threshold(UART_NUM_1, 3);
  esp_sleep_enable_uart_wakeup(UART_NUM_1);

  esp_light_sleep_start();
  wakeup_reason = esp_sleep_get_wakeup_cause();

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  zest_gpioUnhold();

  tNow = millis();

  if (buttonA.read()) noSleepUntilWatermark(tNow + 1000);
  if (buttonB.read()) noSleepUntilWatermark(tNow + 1000);
  if (buttonC.read()) noSleepUntilWatermark(tNow + 1000);

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

  // Disable SD card. Set all pins low and switch off power
  SD.end();
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  digitalWrite(EN_PERI, HIGH);

  backlight(1); // Fade out
  delay(400);

  zest_gpioHold();
  // Also hold backlight pin
  gpio_hold_en(GPIO_NUM_15);    // Backlight

  if (lcdFlip)
  {
    esp_sleep_enable_ext1_wakeup_io(0x80, ESP_EXT1_WAKEUP_ANY_LOW);     // Just IO6 
    rtc_gpio_pulldown_dis(GPIO_NUM_7);
    rtc_gpio_pullup_dis(GPIO_NUM_7);
  }
  else
  {
    esp_sleep_enable_ext1_wakeup_io(0x40, ESP_EXT1_WAKEUP_ANY_LOW);     // Just IO6 
    rtc_gpio_pulldown_dis(GPIO_NUM_6);
    rtc_gpio_pullup_dis(GPIO_NUM_6);
  }
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


zestApp_t app_menu = {NULL, "a", "a", &app_menu_run, &app_menu_draw, NULL, NULL};
zestApp_t app_power = {NULL, "a", "a", &app_power_run, &app_power_draw, NULL, NULL};

zestApp_t mainMenuList[] = 
{
  {&app_menu, "CPM",      "\x8D",   &app_cpm_run,      &app_cpm_draw,       NULL,             NULL},
  {&app_menu, "uSv/h",    "\x64",   &app_usv_run,      &app_usv_draw,       NULL,             NULL},
  {&app_menu, "Spectrum", "\x58",   &app_sp_run,       &app_sp_draw,        NULL,             NULL},
  {&app_menu, "HTTP",     "\xF7",   &app_http_run,     &app_http_draw,      NULL,             &app_http_maySleepFor},
//  {&app_menu, "TCP",      "\xF7",   &app_tcp_run,      &app_tcp_draw,       &app_tcp_stream,  &app_tcp_maySleepFor},
//  {&app_menu, "Endpoint", "\x80",   &app_endpoint_run, &app_endpoint_draw,  NULL,             NULL},
  {&app_menu, "BLE",      "\x5E",   &app_ble_run,      &app_ble_draw,       NULL,             &app_ble_maySleepFor},
  {&app_menu, "SD Log",   "\x80",   &app_log_run,      &app_log_draw,       NULL,             NULL},
//  {&app_menu, "GPS Log",  "\xAF",   &app_gps_run,      &app_gps_draw,       NULL,             &app_gps_maySleepFor},
  {&app_menu, "Settings", "\x81",   &app_settings_run, &app_settings_draw,  NULL,             NULL},
  {&app_menu, "Info",     "\xBC",   &app_info_run,     &app_info_draw,      NULL,             NULL},
  {&app_menu, "Clock",    "\x7B",   &app_clock_run,    &app_clock_draw,     NULL,             NULL},
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

  serviceList[service] = serviceList[service] + 1;
}

void serviceRelease(uint8_t service)
{
  if (serviceList[service] > 0)
  {
    serviceList[service] = serviceList[service] - 1;

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

bool app_menu_run(uint8_t buttonFlags, uint8_t serviceFlags)
{
  static int8_t numApps = sizeof(mainMenuList) / sizeof(zestApp_t);
  bool retVal;

  retVal = false;

  if ((buttonFlags & (1 << BUTTON_1)) != 0)
  {
    // go backwards
    mainMenuSelection--;
    if (mainMenuSelection < 0) mainMenuSelection = numApps - 1;
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_3)) != 0)
  {
    // go forward
    mainMenuSelection++;
    if (mainMenuSelection >= numApps) mainMenuSelection = 0;
    retVal = true;
  }

  if ((buttonFlags & (1 << BUTTON_2)) != 0)
  {
    runningApp = &mainMenuList[mainMenuSelection];
    retVal = true;
  }

  return retVal;
}

void app_menu_draw()
{
  uint8_t i;
  u8g2.setFont(u8g2_font_logisoso16_tf);
  i = u8g2.getStrWidth(mainMenuList[mainMenuSelection].name);
  u8g2.drawStr(64 - (16 + 4 + i)/2 + 16 + 4, 24, mainMenuList[mainMenuSelection].name);
  u8g2.setFont(u8g2_font_open_iconic_all_2x_t);
  u8g2.drawStr(64 - (16 + 4 + i)/2, 24, mainMenuList[mainMenuSelection].icon);
  u8g2.drawTriangle(0, 15, 10, 0, 10, 30);
  u8g2.drawTriangle(127, 15, 117, 0, 117, 30);
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
    zest_gpioUnhold();
    gpio_hold_dis(GPIO_NUM_15);    // Backlight
  }

/*
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
*/
  Serial0.begin(115200);

  // This stops chip from resetting when TeraTerm is closed
  USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
  USB_SERIAL_JTAG.config_update.config_update = 1;

  preferences.begin("pomeloZest", false);
  wifi_ssid = preferences.getString("ssid", ""); 
  wifi_pass = preferences.getString("password", "");
  startup_script = preferences.getString("startup", "");
  preferences.end();

  if (startup_script.length() > 0) startup_script_idx = 0;
  else startup_script_idx = -1;

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
  pinMode(2, OUTPUT);
  pinMode(MEAS, OUTPUT);
  pinMode(BACKLIGHT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(EN_PERI, LOW);
  digitalWrite(2, LOW);
  digitalWrite(MEAS, HIGH);
  digitalWrite(BACKLIGHT, LOW);
  digitalWrite(BUZZER, LOW);

  // SD card SPI not initialized
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);


  u8g2.begin();
  if (lcdFlip) u8g2.setFlipMode(1);
  else u8g2.setFlipMode(0);

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
  buzzerActive = false;
  Timer0 = timerBegin(1000000);
  timerAttachInterrupt(Timer0, timer0_ISR);
  timerStop(Timer0);

  tNow = millis();
  tCanSleepWatermark = tNow;
  tCanSleepExactly = tNow;
  tLightOff = tNow;
  tService = tNow + 1000;
  buttonLockUntil = tNow + 100;

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

  //if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED)
  if ((wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) && (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER))
  {
    // Hard reset
    backlightEnabled = true;
    backlightSeconds = 5;
    backlightIntensity = 50;
    mainMenuSelection = 0;
    usbData = false;
    lcdFlip = false;
    runningApp = &app_menu;
  }

  // If we don't do this CDC port stays in a weird state after reset
  HWCDCSerial.begin();
  if (!usbData) HWCDCSerial.end();


  for (i = 0; i < 8; i++) serviceList[i] = 0;
  serviceRunning = false;
  serviceSkipMeasurement = false;
  serviceUpdateFlags = 0;
  serviceUartPulse = false;

  if (backlightEnabled) backlight(2); // Switch on

  runningApp->run(0, 0);
  u8g2.clearBuffer();
  runningApp->draw();
  u8g2.sendBuffer();

  serialConfig();
  while (serialAvailable()) serialRead();

  // Make pulse char 0xAA (170 decimal),
  // such that it has several transitions
  coreCommand("p16:170\n");

  // If DAQ is stopped it will be restarted. Otherwise
  // it'll let it continue undisturbed
  coreCommand("s");
  startDaq = true;

  // Get status of clicks
  coreCommand("c");

  wifi_connected = false;
  wifi_AP = false;
  wifi_reconnecting = false;
  wifi_tReconnect = 0;
  wifi_mdns = false;
}


void usbPrintDisplay()
{
  uint8_t *scrBuf;
  uint16_t i, j, k;

  if (!(usbData && HWCDCSerial.isConnected())) return;

  scrBuf = u8g2.getBufferPtr();
  HWCDCSerial.write('\n');
  for (k = 0; k < 4; k++)
  {
    for (i = 0; i < 8; i++)
    {
      for (j = 0; j < 128; j++)
      {
        if ((scrBuf[j + k*128] & (1<<i)) != 0)
        {
          HWCDCSerial.write('#');
          HWCDCSerial.write('#');
        }
        else
        {
          HWCDCSerial.write(' ');
          HWCDCSerial.write(' ');
        }
      }
      HWCDCSerial.write('\n');
    }
  }
}


void loop()
{
  static uint16_t bufIdx = 0;
  static uint8_t rxChar;
  static int32_t maySleep, maySleepApp;

  tNow = millis();


  // Button and backlight management
  if (buttonA.read())
  {
    if (backlightEnabled) backlight(2);
    noSleepUntilWatermark(tNow + 1000);
  }
  if (buttonB.read())
  {
    if (backlightEnabled) backlight(2);
    noSleepUntilWatermark(tNow + 1000);
  }
  if (buttonC.read())
  {
    if (backlightEnabled) backlight(2);
    noSleepUntilWatermark(tNow + 1000);
  }

  // Inelegant hack to get rid of first button being
  // processed as a valid command or two
  if (tNow < buttonLockUntil) pressedButtonFlags = 0;

  // Sleep management. Check all sources that could inhibit sleep
  if ((tNow >= tCanSleepWatermark) && (tNow >= tCanSleepExactly) && !usbData)
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

    if (startup_script_idx != -1) maySleep = 0;

    zest_lightsleep(maySleep);
  }

  if (tNow >= tService)
  {
    tService += 1000;
    serviceRunning = serviceCall(true);
    tServiceWait = tNow + 150;
    if (serviceList[SERVICE_TICK] != 0) serviceUpdateFlags |= (1 << SERVICE_TICK);

    if (startup_script_idx >= 0)
    {
      // run one instruction -- a button press
      switch(startup_script[startup_script_idx])
      {
        case '1':
          pressedButtonFlags |= (1 << BUTTON_1);
          break;
        case '2':
          pressedButtonFlags |= (1 << BUTTON_2);
          break;
        case '3':
          pressedButtonFlags |= (1 << BUTTON_3);
          break;
        case '4':
          pressedButtonFlags |= (1 << BUTTON_1_LONG);
          break;
        case '5':
          pressedButtonFlags |= (1 << BUTTON_2_LONG);
          break;
        case '6':
          pressedButtonFlags |= (1 << BUTTON_3_LONG);
          break;
        default:
          startup_script_idx = -2;
          break;
      }
      startup_script_idx++;
    }
  }
  
  if (serviceRunning)
  {
    if (tNow >= tServiceWait)
    {
      serviceRunning = serviceCall(false);
      tServiceWait = tNow + 150;
    }
  }

  if ((pressedButtonFlags & (1 << BUTTON_1_LONG)) != 0)
  {
    if (app_power.parent == NULL)
    {
      app_power.parent = runningApp;
      runningApp = &app_power;
    }
  }

  if (app_power.parent != NULL)
  {
    bool redraw1, redraw2;
    // Keep running app underneath
    redraw1 = app_power.parent->run(0, serviceUpdateFlags);
    redraw2 = app_power.run(pressedButtonFlags, serviceUpdateFlags);

    if (redraw1 || redraw2)
    {
      u8g2.clearBuffer();
      if (app_power.parent != NULL) app_power.parent->draw();
      runningApp->draw();
      u8g2.sendBuffer();
    }
  }
  else
  {
    if (runningApp->run(pressedButtonFlags, serviceUpdateFlags))
    {
      u8g2.clearBuffer();
      runningApp->draw();
      u8g2.sendBuffer();
    }
  }
  serviceUpdateFlags = 0;
  pressedButtonFlags = 0;

  if ((tNow >= tLightOff) && (backlightSeconds != 0))
  {
    backlight(1); // Fade out
  }

  while (GPS.read() != 0) ;
  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
    if (strncmp(GPS.lastNMEA(), "$GNGGA", 6) == 0)
    {
      if ((int)GPS.fix != 0)  // Only let app know if we have a fix
      {
        if (serviceList[SERVICE_GPS] != 0) serviceUpdateFlags |= (1 << SERVICE_GPS);
      }
    }
  }


  if (usbData)
  {
    if (HWCDCSerial.available())
    {
      switch(HWCDCSerial.read())
      {
        case 'a':
          pressedButtonFlags |= (1 << BUTTON_1);
          break;
        case 's':
          pressedButtonFlags |= (1 << BUTTON_2);
          break;
        case 'd':
          pressedButtonFlags |= (1 << BUTTON_3);
          break;
        case 'q':
          pressedButtonFlags |= (1 << BUTTON_1_LONG);
          break;
        case 'w':
          pressedButtonFlags |= (1 << BUTTON_2_LONG);
          break;
        case 'e':
          pressedButtonFlags |= (1 << BUTTON_3_LONG);
          break;
        case 'r':
          usbPrintDisplay();
          break;
      }
    }
  }

  if (serialAvailable())
  {
    noSleepUntilExactly(tNow + 250);
    if (runningApp->stream != NULL)
    {
      while (serialAvailable())
      {
        if (!runningApp->stream(serialRead()))
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
          noSleepUntilExactly(tNow + 10);
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
  JsonDocument doc;
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
  if (buttonA.read()) noSleepUntilWatermark(tNow + 1000);
  if (buttonB.read()) noSleepUntilWatermark(tNow + 1000);
  if (buttonC.read()) noSleepUntilWatermark(tNow + 1000);

  if (doc["type"] == "dosimetry") parseDataDosimetry(&doc);
  if (doc["type"] == "spectrum") parseDataSpectrum(&doc);
  if (doc["type"] == "config") parseDataConfig(&doc);
  if (doc["type"] == "system") parseDataSystem(&doc);
}


void parseDataDosimetry(JsonDocument *doc)
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

void parseDataConfig(JsonDocument *doc)
{
  uint32_t uartPulse;
  uartPulse = (*doc)["payload"]["outputs"]["uart"]["fastPulse"].as<int>();
  if (uartPulse != 0) serviceUartPulse = true;
  else serviceUartPulse = false;
  serviceUpdateFlags |= (1 << SERVICE_CONFIG);
}

void parseDataSystem(JsonDocument *doc)
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

void parseDataSpectrum(JsonDocument *doc)
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

  if (spectrumMax == 0) spectrumMax = 1;

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
  sprintf(printBuf, "%lu", serviceHistCounts);
  u8g2.drawStr(80, 10, printBuf);
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
    delay(500);
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
