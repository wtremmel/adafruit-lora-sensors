/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <ArduinoLog.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <RTCZero.h>

// #define DEBUG 1

#include <CayenneLPP.h>
CayenneLPP lpp(51);


// Sensor Libraries
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"
#include <ArduinoECCX08.h>
#include "Adafruit_ADS1115"

// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_ADS1115 ads1115;

Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
RTCZero rtc;

bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool ecc508_found= false;
bool voltage_found= true;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;
bool adc1115_found = false;

bool setup_complete = false;
bool led_dynamic = true; // LED shows if system is asleep or not

unsigned const rainPin = A1;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x8A, 0x30, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x23, 0x34, 0x64, 0x9B, 0x05, 0x7E, 0x58, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x40, 0x87, 0x47, 0xE0, 0xEC, 0x04, 0xCC, 0x0B, 0x98, 0x72, 0x95, 0x08, 0xB8, 0x61, 0xC2, 0x4A };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
static unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
.nss = 8,
.rxtx = LMIC_UNUSED_PIN,
.rst = 4,
//.dio = {3, 6, 11},
.dio = {3, 6, LMIC_UNUSED_PIN},
};

// forward declarations
void setup_I2C(void);

void setup_serial() {
  Serial.begin(115200);
#if DEBUG
  while (!Serial);
#endif
}

// ----------- Battery stuff
#define VBATPIN A7

float my_voltage() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024*4; // convert to voltage

  // LiPo battery is really strange
  if (measuredvbat > 3.57 && measuredvbat < 3.64) {
    TX_INTERVAL = 240;
  } else {
    TX_INTERVAL = 60;
  }

  return measuredvbat;
}

// --------RTC Stuff-----------------

void rtcAlarm() {
  rtc_alarm_raised = true;
}

void setup_RTC () {
  rtc.begin();
  rtc.setEpoch(0);
  rtc.attachInterrupt(rtcAlarm);
  rtc_init_done = true;
  pinMode(LED_BUILTIN, OUTPUT);
  if (led_dynamic) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void sleepfor(int seconds) {
  uint32_t now = rtc.getEpoch();

  if (!setup_complete)
    return;

  Log.verbose(F("entering sleepfor(%d)"),seconds);
  rtc.setAlarmEpoch(now + seconds);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  if (led_dynamic)
    digitalWrite(LED_BUILTIN, LOW);
  Serial.end();
  USBDevice.detach(); // Safely detach the USB prior to sleeping
  rtc.standbyMode();    // Sleep until next alarm match
  USBDevice.attach();
  setup_serial();

  if (led_dynamic)
    digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);
  Log.verbose(F("leaving sleepfor(%d)"),seconds);
}

void read_tsl2561() {
  sensors_event_t event;
  Log.verbose(F("read_tsl2561"));
  tsl2561.getEvent(&event);
  lpp.addLuminosity(4,event.light);
}

void read_si7021() {
  Log.verbose(F("read_si70721"));
  lpp.addTemperature(1, si7021.readTemperature());
  lpp.addRelativeHumidity(2, si7021.readHumidity());
}

void read_bme280() {
  Log.verbose(F("read_bme280"));
  lpp.addTemperature(1,bme280.readTemperature());
  lpp.addRelativeHumidity(2,bme280.readHumidity());
  lpp.addBarometricPressure(3,bme280.readPressure() / 100.0F);
}

void read_voltage() {
  float v = my_voltage();
  lpp.addAnalogInput(5,v);
}

extern "C" char *sbrk(int i);
void read_ram() {
  char stack_dummy = 0;
  lpp.addDigitalInput(7, &stack_dummy - sbrk(0));
}

void read_rain() {
  pinMode(rainPin,INPUT_PULLUP);
  delay(1000);
  unsigned int r = analogRead(rainPin);
  lpp.addDigitalInput(6,4096-r);
  pinMode(rainPin,INPUT);
}

void readSensors() {
  lpp.reset();
  setup_I2C();

  if (si7021_found) {
    read_si7021();
  }
  if (bme280_found) {
    read_bme280();
  }
  if (tsl2561_found) {
    read_tsl2561();
  }
  if (voltage_found) {
    read_voltage();
  }
  read_rain();
  // read_ram();
}



void do_send(osjob_t* j){

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      (F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

      sleepfor(TX_INTERVAL);

      lpp.reset();
      readSensors();

      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
      Log.verbose(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
    setup_complete = true;
}

// -------------- Command Processing -----------------
void process_system_led_command(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  switch (buffer[0]) {
    case 0:
      led_dynamic = false;
      pinMode(LED_BUILTIN,LOW);
      break;
    case 1:
      led_dynamic = false;
      pinMode(LED_BUILTIN, HIGH);
      break;
    case 0xff:
      led_dynamic = true;
      break;
    default:
      Log.error(F("Unknown LED command %d"), buffer[0]);
      break;
  }
}

void process_system_set_epoch(unsigned char len, unsigned char *buffer) {
  if (len != 4) {
    Log.error(F("Epoch has wrong len (%d != 4)"),len);
    return;
  }
  rtc.setEpoch((uint32_t) buffer);
}

void process_system_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length system command"));
    return;
  }
  switch (buffer[0]) {
    case 0x03:
      process_system_led_command(len-1,buffer+1);
      break;
    case 0x04:
      process_system_set_epoch(len-1,buffer+1);
      break;
  }
}

void process_sensor_command(unsigned char len, unsigned char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length sensor command"));
    return;
  }
}

void process_received_lora(unsigned char len, unsigned char *buffer) {
  if (len == 0)
    return;

  Log.verbose(F("Processing %d bytes of received data"),len);
  switch (buffer[0]) {
    case 0:
      process_system_command(len-1,buffer+1);
      break;
    case 1:
      process_sensor_command(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown command %d"),buffer[0]);
      break;
  }
}

// -----------------------
void onEvent (ev_t ev) {
    Log.verbose(F("onEvent:"));

    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Log.verbose(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Log.verbose(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Log.verbose(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Log.verbose(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Log.verbose(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Log.verbose(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Log.verbose(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Log.verbose(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Log.verbose(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Log.verbose(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Log.verbose(F("Received ack"));
            if (LMIC.dataLen) {
              Log.verbose(F("Received %d bytes of payload"),LMIC.dataLen);
              process_received_lora(LMIC.dataLen,LMIC.frame);
            }
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            break;
        case EV_LOST_TSYNC:
            Log.verbose(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Log.verbose(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Log.verbose(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Log.verbose(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Log.verbose(F("EV_LINK_ALIVE"));
            break;
         default:
            Log.verbose(F("Unknown event"));
            break;
    }
}



//
// Scan for sensors
//
void setup_I2C() {
  byte error, address;
  unsigned int devices=0;

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 - 0x4b 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)

  Log.verbose("Scanning i2c bus");
  Wire.begin();
  Wire.setClock(10000);
  for(address = 1; address < 127; address++ ) {
    Log.verbose(F("Trying 0x%x"),address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission(false);

    if (error == 0) {
      Log.verbose(F("I2C device found at address 0x%x !"),address);
      devices++;

      if ((address == 0x39) || (address == 0x29) || (address == 0x49)) {
        tsl2561 = Adafruit_TSL2561_Unified(address);
        tsl2561_found = tsl2561.begin();
        Log.verbose(F("TSL2561 found? %T"),tsl2561_found);
        if (tsl2561_found) {
          // init the sensor
          tsl2561.enableAutoRange(true);
          tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
        }
      }
      if (address == 0x40) {
        // SI7021
        si7021 = Adafruit_Si7021();
        si7021_found = si7021.begin();
        Log.verbose(F("Si7021 found? %T"),si7021_found);
      }

      if ((address >= 0x48) && (address <= 0x4b)) {
        ads1115 = Adafruit_ADS1115(address);
        ads1115.begin();
        ads1115_found = true;
        Log.notice(F("ADS1115 found at 0x%x"),address);
      }

      if (address == 0x60) {
        // ECC508
        ecc508_found = ECCX08.begin();
        Log.verbose(F("ECC508 found? %T"),ecc508_found);
      }

      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
        Log.verbose(F("BME280 found? %T"),bme280_found);
      }
    }
  }
  Log.verbose(F("i2c bus scanning complete, %d devices"),devices);
}

// Logging helper routines
void printTimestamp(Print* _logOutput) {
  char c[12];
  sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
  _logOutput->print('\n');
}


void setup_logging() {
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

void setup() {
  setup_serial();
  delay(5000);

  setup_logging();

  setup_RTC();
  // setup_I2C(); // happens when sensors are read


  // setup Rain detector
  analogReadResolution(12);


    // LMIC init
  os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();


    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    setup_complete = true;

}


void loop() {
  setup_complete = true;
  // Log.verbose(F("entering main loop"));
  os_runloop_once();
}
