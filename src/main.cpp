//  Based on https://github.com/RAKWireless/WisBlock/blob/master/examples/RAK4630/communications/LoRa/LoRaWAN/LoRaWAN_OTAA_ABP/LoRaWAN_OTAA_ABP.ino
//  Note: This program needs SX126x-Arduino Library version 2.0.0 or later. In platformio.ini, set...
//    lib_deps = beegee-tokyo/SX126x-Arduino@^2.0.0

/**
 * @file LoRaWAN_OTAA_ABP.ino
 * @author rakwireless.com
 * @brief LoRaWan node example with OTAA/ABP registration
 * @version 0.1
 * @date 2020-08-21
 *
 * @copyright Copyright (c) 2020
 *
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */
#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include <Wire.h>
#include <NCP5623.h>

// RAK4630 supply two LEDs
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

NCP5623 rgb;

bool doOTAA = true;                                               // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                                       /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_1                                     /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5                               /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 5                                        /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_C;                           /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;           /* Region:US915*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;               /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                              /* data port*/

/**@brief Structure containing LoRaWAN parameters, needed for lmh_init()
 */
static lmh_param_t g_lora_param_init = {
    LORAWAN_ADR_ON,
    LORAWAN_DATERATE,
    LORAWAN_PUBLIC_NETWORK,
    JOINREQ_NBTRIALS,
    LORAWAN_TX_POWER,
    LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWAN callback functions, needed for lmh_init()
 */
static lmh_callback_t g_lora_callbacks = {
    BoardGetBatteryLevel,
    BoardGetUniqueId,
    BoardGetRandomSeed,
    lorawan_rx_handler,
    lorawan_has_joined_handler,
    lorawan_confirm_class_handler,
    lorawan_join_failed_handler};

// OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private definitions
#define LORAWAN_APP_DATA_BUFF_SIZE 64                                         /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                                            /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

void setup()
{
  // enable RAK14001
  pinMode(WB_IO6, OUTPUT);
  digitalWrite(WB_IO6, HIGH);

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

  // If using Native I2C
  Wire.begin();
  Wire.setClock(100000);

  // Initialize LoRa chip.
  lora_rak4630_init();

  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWAN!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
  case LORAMAC_REGION_AS923:
    Serial.println("Region: AS923");
    break;
  case LORAMAC_REGION_AU915:
    Serial.println("Region: AU915");
    break;
  case LORAMAC_REGION_CN470:
    Serial.println("Region: CN470");
    break;
  case LORAMAC_REGION_CN779:
    Serial.println("Region: CN779");
    break;
  case LORAMAC_REGION_EU433:
    Serial.println("Region: EU433");
    break;
  case LORAMAC_REGION_IN865:
    Serial.println("Region: IN865");
    break;
  case LORAMAC_REGION_EU868:
    Serial.println("Region: EU868");
    break;
  case LORAMAC_REGION_KR920:
    Serial.println("Region: KR920");
    break;
  case LORAMAC_REGION_US915:
    Serial.println("Region: US915");
    break;
  case LORAMAC_REGION_RU864:
    Serial.println("Region: RU864");
    break;
  case LORAMAC_REGION_AS923_2:
    Serial.println("Region: AS923-2");
    break;
  case LORAMAC_REGION_AS923_3:
    Serial.println("Region: AS923-3");
    break;
  case LORAMAC_REGION_AS923_4:
    Serial.println("Region: AS923-4");
    break;
  }
  Serial.println("=====================================");

  if (!rgb.begin())
  {
    Serial.println("RAK14001 not found on the I2C line");
    while (1)
      ;
  }
  else
  {
    Serial.println("RAK14001 Found. Beginning execution");
  }

  // create a user timer to send data to server period
  uint32_t err_code;
  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWAN stack
  err_code = lmh_init(
      &g_lora_callbacks,
      g_lora_param_init,
      doOTAA,
      g_CurrentClass,
      g_CurrentRegion);

  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
}

/**
 * LED light up Gradually
 *
 * @param red          0-255
 * @param green        0-255
 * @param blue         0-255
 * @param stepNum      0-31
 * @param msPerStep    1ms-248ms
 */
void gradualLightUp(uint8_t red, uint8_t green, uint8_t blue, uint8_t stepNum, uint8_t msPerStep)
{
  rgb.setColor(red, green, blue);

  // set dimming up end, range is 1 to 30
  rgb.setGradualDimmingUpEnd(stepNum);

  // set dimming step ms, range is 1 ms to 248 ms
  rgb.setGradualDimming(msPerStep);

  delay(stepNum * msPerStep);
}

/**
 * LED dark down Gradually
 *
 * @param red          0-255
 * @param green        0-255
 * @param blue         0-255
 * @param stepNum      0-31
 * @param msPerStep    1ms-248ms
 */
void gradualDarkDown(uint8_t red, uint8_t green, uint8_t blue, uint8_t stepNum, uint8_t msPerStep)
{
  rgb.setColor(red, green, blue);

  // set dimming up end, range is 1 to 30
  rgb.setGradualDimmingDownEnd(31 - stepNum);

  // set dimming step ms, range is 1 ms to 248 ms
  rgb.setGradualDimming(msPerStep);

  delay(stepNum * msPerStep);
}

void loop()
{
  // // Put your application tasks here, like reading of sensors,
  // // Controlling actuators and/or other functions.
  // // RED
  // gradualLightUp(100, 0, 0, 30, 100);
  // gradualDarkDown(100, 0, 0, 30, 100);

  // // GREEN
  // gradualLightUp(0, 100, 0, 30, 100);
  // gradualDarkDown(0, 100, 0, 30, 100);

  // // BLUE
  // gradualLightUp(0, 0, 100, 30, 100);
  // gradualDarkDown(0, 0, 100, 30, 100);

  // // YELLOW
  // gradualLightUp(100, 100, 0, 30, 100);
  // gradualDarkDown(100, 100, 0, 30, 100);

  // // PURPLE
  // gradualLightUp(100, 0, 255, 30, 100);
  // gradualDarkDown(100, 0, 255, 30, 100);

  // // CYAN
  // gradualLightUp(0, 100, 100, 30, 100);
  // gradualDarkDown(0, 100, 100, 30, 100);

  // // WHITE
  // gradualLightUp(100, 100, 100, 30, 100);
  // gradualDarkDown(100, 100, 100, 30, 100);
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  if (doOTAA == true)
  {
    Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
  }

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}

/**@brief LoRa function for handling OTAA join failed
 */
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUIs and Keys!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief Function for handling LoRaWAN received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 *
 *  Command	  Base64	  HEX     Function
 *  G0	      RzA=	    4730    Switch green LED off
 *  G1	      RzE=	    4731    Switch green LED on
 *  B0	      QjA=	    4230    Switch blue LED off
 *  B1	      QjE=	    4231    Switch blue LED on
 *  R0	      UjA=	    5230    Switch red LED off
 *  R1	      UjE=	    5231    Switch red LED on
 *
 *  SC        U0M=      5343    Set RGB LED Color
 *
 * e.g.: set RGB LED (HEX):
 * 5343 RED GREEN BLUE
 * RED:   5343FF0000
 * GREEN: 534300FF00
 * BLUE:  53430000FF
 *
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

  for (int idx = 0; idx < app_data->buffsize; idx++)
  {
    Serial.printf("0x%0X", app_data->buffer[idx]);
    Serial.println();
  }
  Serial.println();

  if (app_data->buffer[0] == 'G')
  {
    // Handle Green LED
    if (app_data->buffer[1] == '0')
    {
      // Switch LED off
      digitalWrite(LED_GREEN, LOW);
    }
    else
    {
      // Switch LED on
      digitalWrite(LED_GREEN, HIGH);
    }
  }
  if (app_data->buffer[0] == 'B')
  {
    // Handle BLUE LED
    if (app_data->buffer[1] == '0')
    {
      // Switch LED off
      digitalWrite(LED_BLUE, LOW);
    }
    else
    {
      // Switch LED on
      digitalWrite(LED_BLUE, HIGH);
    }
  }
  if (app_data->buffer[0] == 'S')
  {
    if (app_data->buffer[1] == 'C')
    {
      // Serial.println("Set RGB LED Color");
      rgb.setColor(app_data->buffer[2], app_data->buffer[3], app_data->buffer[4]);
      rgb.setGradualDimmingUpEnd(30);
      rgb.setGradualDimming(100);
      rgb.setGradualDimmingDownEnd(1);
      rgb.setGradualDimming(100);
      delay(30 * 100);
    }
  }
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    // Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  // m_lora_app_data.buffer[i++] = 'H';
  // m_lora_app_data.buffer[i++] = 'e';
  // m_lora_app_data.buffer[i++] = 'l';
  // m_lora_app_data.buffer[i++] = 'i';
  // m_lora_app_data.buffer[i++] = 'u';
  // m_lora_app_data.buffer[i++] = 'm';
  m_lora_app_data.buffsize = i;

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}

/**@brief Function for handling user timerout event.
 */
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  // Serial.println("Sending frame now...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}