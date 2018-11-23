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

// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
RTCZero rtc;

bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool ecc508_found= false;
bool voltage_found= true;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;

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

static uint8_t mydata[] = "Hello, world!";
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


// ----------- Battery stuff
#define VBATPIN A7

float my_voltage() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  if (measuredvbat > 3.6) {
    TX_INTERVAL = 60;
  } else {
    TX_INTERVAL = 120;
  }
  
  return measuredvbat;
}

// --------RTC Stuff-----------------

void rtcAlarm() {
  rtc_alarm_raised = true;
}

void setupRTC () {
  rtc.begin();
  rtc.setEpoch(0);
  rtc.attachInterrupt(rtcAlarm);
  rtc_init_done = true;
#if DEBUG
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
#endif
}

void sleepfor(int seconds) {
  uint32_t now = rtc.getEpoch();
  rtc.setAlarmEpoch(now + seconds);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
#if DEBUG
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
#endif
  rtc.standbyMode();

#if DEBUG
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
#endif

}

// -----------------------
void onEvent (ev_t ev) {
#if DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
#endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
#if DEBUG
            Serial.println(F("EV_SCAN_TIMEOUT"));
#endif
            break;
        case EV_BEACON_FOUND:
#if DEBUG
            Serial.println(F("EV_BEACON_FOUND"));
#endif
            break;
        case EV_BEACON_MISSED:
#if DEBUG
            Serial.println(F("EV_BEACON_MISSED"));
#endif
            break;
        case EV_BEACON_TRACKED:
#if DEBUG
            Serial.println(F("EV_BEACON_TRACKED"));
#endif
            break;
        case EV_JOINING:
#if DEBUG
            Serial.println(F("EV_JOINING"));
#endif
            break;
        case EV_JOINED:
#if DEBUG
            Serial.println(F("EV_JOINED"));
#endif

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
#if DEBUG
            Serial.println(F("EV_RFU1"));
#endif
            break;
        case EV_JOIN_FAILED:
#if DEBUG
            Serial.println(F("EV_JOIN_FAILED"));
#endif
            break;
        case EV_REJOIN_FAILED:
#if DEBUG
            Serial.println(F("EV_REJOIN_FAILED"));
#endif
            break;
            break;
        case EV_TXCOMPLETE:
#if DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
#endif
            if (LMIC.txrxFlags & TXRX_ACK)
#if DEBUG
              Serial.println(F("Received ack"));
#endif
            if (LMIC.dataLen) {
#if DEBUG
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
#endif
            }
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            break;
        case EV_LOST_TSYNC:
#if DEBUG
            Serial.println(F("EV_LOST_TSYNC"));
#endif
            break;
        case EV_RESET:
#if DEBUG
            Serial.println(F("EV_RESET"));
#endif
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
#if DEBUG
            Serial.println(F("EV_RXCOMPLETE"));
#endif
            break;
        case EV_LINK_DEAD:
#if DEBUG
            Serial.println(F("EV_LINK_DEAD"));
#endif
            break;
        case EV_LINK_ALIVE:
#if DEBUG
            Serial.println(F("EV_LINK_ALIVE"));
#endif
            break;
         default:
#if DEBUG
            Serial.println(F("Unknown event"));
#endif
            break;
    }
}


void read_tsl2561() {
  sensors_event_t event;
  tsl2561.getEvent(&event);
  lpp.addLuminosity(4,event.light);
}

void read_si7021() {
  lpp.addTemperature(1, si7021.readTemperature());
  lpp.addRelativeHumidity(2, si7021.readHumidity());
}

void read_bme280() {
  lpp.addTemperature(1,bme280.readTemperature());
  lpp.addRelativeHumidity(2,bme280.readHumidity());
  lpp.addBarometricPressure(3,bme280.readPressure() / 100.0F);
}

void read_voltage() {
  float v = my_voltage();

  if (v <= 4.3) { // do not send if connected to USB
    lpp.addAnalogInput(5,v);
  }
}

void readSensors() {
  lpp.reset();
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
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
#if DEBUG      
        Serial.println(F("OP_TXRXPEND, not sending"));
#endif
    } else {
        // Prepare upstream data transmission at the next possible time.

      sleepfor(TX_INTERVAL);

      lpp.reset();
      readSensors();
         
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
#if DEBUG      
      Serial.println(F("Packet queued"));
#endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


//
// Scan for sensors
//
void setupI2C() {
  byte error, address;
  int nDevices;

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)

#if DEBUG
  Serial.println("Scanning i2c bus");
#endif
  Wire.begin();
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
#if DEBUG
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
#endif

      if (address == 0x39) {
        tsl2561 = Adafruit_TSL2561_Unified(address);
        tsl2561_found = tsl2561.begin();
#if DEBUG
        Serial.print("TSL2561 found? ");
        Serial.println(tsl2561_found);
#endif
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
#if DEBUG
        Serial.print("Si7021 found? ");
        Serial.println(si7021_found);
#endif
      }

      if (address == 0x60) {
        // ECC508
        ecc508_found = ECCX08.begin();
#if DEBUG
        Serial.print("ECC508 found? ");
        Serial.println(ecc508_found);
#endif
      }
      
      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
#if DEBUG
        Serial.print("BME280 found? ");
        Serial.println(bme280_found);
#endif
      }
    }
  }
}

void setup() {
#if DEBUG
    Serial.begin(115200);
    while (!Serial);
    Serial.println(F("Starting"));
    delay(2000);
#else
    delay(5000);
#endif

    setupRTC();
    setupI2C();


    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}


void loop() {
  os_runloop_once();
}
