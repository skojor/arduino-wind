/*  
  Davis anemometer 6410 wireless sensor using an Arduino pro mini (328p) running at 8 MHz internal clock. 
  The 328p has the Optiboot bootloader installed to enable the watchdog, and also allows
  the 16 MHz pro mini board to be configured as 8 MHz 3.3V.

  The anemometer is connected to a 5.1k pull-up resistor on the wind speed wire, 
  a resistor network to keep the wind direction voltage approx. 0-1 VDC 
  (ADC is running on internal 1.1V reference voltage), 
  and a couple of ceramic capacitors and TVS diodes to reduce static interference.

  A DS18B20 sensor is attached to report temperature.

  The communication is using the MySensors library v2.3.2, and an RFM69HW radio chip. 
  Data is sent in passive mode, as this seems to be more reliable over time.

  The unit is powered with a 18650 cell via a low power MCP1702 3.3V regulator, and charged 
  with a solar panel. The battery voltage is monitored with a voltage divider. 
  */

#include <arduino.h>

//#define TEST_MODE
#define FREQ_ERROR 0L // added to base freq

#ifdef TEST_MODE
#define TEST_DEBUG true
//#define MY_DEBUG_VERBOSE_RFM69
#define MY_DEBUG
#else
#define TEST_DEBUG false
#endif

#define CHILD_ID_DIRECTION 0
#define CHILD_ID_SPEED     1
#define CHILD_ID_GUST      2
#define CHILD_ID_VOLTAGE   3
#define CHILD_ID_TEMP      4

#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER
#define MY_BAUD_RATE 38400
#define MY_PASSIVE_NODE

#if __has_include("MySensors_wind_radio_private.h")
#include "MySensors_wind_radio_private.h"
#endif

#ifndef MY_RFM69_FREQUENCY
#define MY_RFM69_FREQUENCY 868000000L + FREQ_ERROR
#endif
#ifndef MY_RFM69_NETWORKID
#define MY_RFM69_NETWORKID 100
#endif
#ifndef MY_RFM69_ENABLE_ENCRYPTION
// #define MY_RFM69_ENABLE_ENCRYPTION
#endif
#ifndef MY_NODE_ID
#define MY_NODE_ID 1
#endif
#ifndef MY_RFM69_TX_POWER_DBM
#define MY_RFM69_TX_POWER_DBM 7
#endif

const unsigned int WIND_SPD_CALC_INTERVAL = 5; // Calc. wind speed every x second
const unsigned int  SEND_SPD_INTERVAL = 5; // how often to send wind speed (sec)
const unsigned int  SEND_DIR_INTERVAL = 30; // wind dir sent here
const unsigned int  SEND_BATTERY_INTERVAL = 1800; // how often to send battery data (sec)
const unsigned int  SEND_TEMP_INTERVAL = 120;
const byte BATTERY_SENSE_PIN = A3;
const byte WDIR_SENSE = A0;
const byte WSPD_SENSE = 3;
const byte ONE_WIRE_BUS = A1;
const float R1  = 569;    // Vcc - BATTERY_SENSE_PIN (voltage divider)
const float R2  = 77;    // BATTERY_SENSE_PIN - GND
const float Rratio = (R1 + R2) / R2;
const float battLow =  2.7; // Minimum voltage on 18650
const float battHigh = 4.20;

#include <time.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MySensors.h>

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tSensors(&oneWire);
DeviceAddress tempsensor;

float wSpd, vBat, temp;
uint8_t wDir;
bool flip = true;
int batteryPcnt;
volatile uint16_t wspdCtr = 0; // Max wspdCtr = 80 Hz * WIND_SPD_CALC_INTERVAL (5 s) = 400
uint32_t wspdCalcTime, wspdTime, wdirTime, batTime, tempTime;
bool tempSensorValid;
bool tempRequestPending = false;
uint32_t tempRequestTime = 0;
int oldWdir = -1, oldWspd = -1, oldTemp = 0, oldBat = 0;
volatile uint32_t seconds = 0; 
volatile uint8_t tick12ms = 0;
volatile uint8_t prevTick = 0;
uint32_t now = 0;
const uint32_t RESET_INTERVAL_SECONDS = 21600; // 6 hours

MyMessage msgDirection(CHILD_ID_DIRECTION, V_DIRECTION);
MyMessage msgSpeed(CHILD_ID_SPEED, V_WIND);
MyMessage msgGust(CHILD_ID_GUST, V_GUST);
MyMessage msgVoltage(CHILD_ID_VOLTAGE, V_VOLTAGE);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgPrefix1(CHILD_ID_SPEED, V_UNIT_PREFIX);  // Custom unit message.
MyMessage msgPrefix2(CHILD_ID_GUST, V_UNIT_PREFIX);  // Custom unit message.

const float MAX_WIND_SPEED = 65.0;
const float MAX_JUMP = 8.0;
const float MAX_FACTOR = 2.8;
const float ALPHA = 0.15;

constexpr uint8_t TIMER2_PRELOAD = 158;
constexpr uint8_t TICKS_PER_SECOND = 80;
uint16_t WDIR_ADC_MAX = 870;

float windAvg = 0.0;
bool avgInitialized = false;
uint16_t randWaitWspdTest = 0;

void readBatVoltage();
void calcWspd(int timediff);
bool isWindValid(float newvalue, float oldvalue);
void pciSetup(byte pin);
void mySleep();
void wspdISR();
void performReset();
inline void adcOn()  { ADCSRA |=  (1<<ADEN); }
inline void adcOff() { ADCSRA &= ~(1<<ADEN); }

void presentation() {
  sendSketchInfo("Wind sensor", "2.4");
  present(CHILD_ID_DIRECTION, S_WIND);
  present(CHILD_ID_SPEED, S_WIND);
  present(CHILD_ID_GUST, S_WIND);
  present(CHILD_ID_VOLTAGE, S_MULTIMETER);
  present(CHILD_ID_TEMP, S_TEMP);
}

void setup() {
  // Clean startup in case the previous reset was watchdog-triggered.
  MCUSR = 0;
  wdt_disable();

  analogReference(INTERNAL);
  randomSeed(analogRead(A2));
  pinMode(WSPD_SENSE, INPUT);
  pinMode(WDIR_SENSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(WSPD_SENSE), wspdISR, FALLING);

  send(msgPrefix1.set("m/s"));
  send(msgPrefix2.set("m/s"));

  tSensors.begin();
  tempSensorValid = tSensors.getAddress(tempsensor, 0);

  if (tempSensorValid) {
    tSensors.setResolution(11);
    tSensors.setWaitForConversion(false);
  }

  for (int i=0; i<3; i++) { 
    readBatVoltage();
    Serial.println(vBat);
    sleep(250);
  }
  timer2Setup();
  wdt_enable(WDTO_8S);
}

void before() { // Må gjøres FØR init av MySensors!
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  delay(10);
  digitalWrite(9, LOW);
  delay(10); // viktig: la radioen starte opp igjen
}

void loop() {
  wdt_reset();

#ifdef TEST_MODE
  randWaitWspdTest = random(10, 20);
  if (tick12ms % randWaitWspdTest == 0) { // Random ms passed, simulating wspd.
    wspdISR();
  }
#endif
  
  uint32_t sec;
  noInterrupts();
  sec = seconds;
  interrupts();

  if (sec >= RESET_INTERVAL_SECONDS) performReset();

  if (sec != now) {
    now = sec;
    
    if (now % 120 == 0) oldWspd = oldTemp = oldWdir = random(); // To resend all data every now and then no matter what

    if (now - wspdCalcTime >= WIND_SPD_CALC_INTERVAL) {
      calcWspd(now - wspdCalcTime);   
      wspdCalcTime = now;
    }

    if (now - wspdTime >= SEND_SPD_INTERVAL) {
      if (!isnan(wSpd) && wSpd >= 0 && wSpd < MAX_WIND_SPEED && (int)(wSpd * 10) != oldWspd) {
        wdt_reset();
        transportReInitialise();
        oldWspd = (int)(wSpd * 10);
        send(msgSpeed.set(wSpd, 1));
        transportDisable();
        wdt_reset();
      }
      wspdTime = now;
    }

    else if (now - wdirTime >= SEND_DIR_INTERVAL) {
      adcOn();
      delay(10);
      uint16_t wdir_adc = analogRead(WDIR_SENSE);
      adcOff();

      //if (wdir_adc > WDIR_ADC_MAX && wdir_adc < 900) WDIR_ADC_MAX = wdir_adc; // Can be removed, corrects variations in max voltage on wdir input
      wDir = map(wdir_adc, 0, WDIR_ADC_MAX, 0, 359);
      
      if (wDir > 359) wDir = 359;
        
      if ((int)(wDir) != oldWdir) {
        wdt_reset();
        transportReInitialise();
        send(msgDirection.set(wDir));
        //smartSleep(1);
        oldWdir = wDir;
        transportDisable();
        wdt_reset();
      }
      wdirTime = now;
    }

    else if (now - batTime >= SEND_BATTERY_INTERVAL) {
      readBatVoltage();
      wdt_reset();
      transportReInitialise();
      if (flip) {
        send(msgVoltage.set(vBat, 2));
        oldBat = (int)(vBat * 100);
      }
      else
        sendBatteryLevel(batteryPcnt);
  
      flip = !flip;
      transportDisable();
      wdt_reset();
      
      batTime = now;
    }

    else if (tempSensorValid) {
      if (!tempRequestPending && now - tempTime >= SEND_TEMP_INTERVAL) {
        tSensors.requestTemperatures();
        tempRequestTime = now;
        tempRequestPending = true;
      }

      if (tempRequestPending && now != tempRequestTime) {
        temp = tSensors.getTempC(tempsensor);
        if ((int)(temp * 10) != oldTemp) {
          wdt_reset();
          transportReInitialise();
          send(msgTemp.set(temp, 1));
          //smartSleep(1);
          oldTemp = (int)(temp * 10);
          transportDisable();
          wdt_reset();
        }
        tempTime = now;
        tempRequestPending = false;
      }
    }
  }
  mySleep();
}

void readBatVoltage() {
  adcOn();
  delay(10);
  vBat = Rratio * (float)analogRead(BATTERY_SENSE_PIN) * 1.1 / 1023.0;
  adcOff();
  batteryPcnt = (vBat - battLow) * 100 / (battHigh - battLow);
  if (batteryPcnt < 0) batteryPcnt = 0;
  else if (batteryPcnt > 100) batteryPcnt = 100;
}

void calcWspd(int timediff) {
  uint16_t pulses;
  noInterrupts();
  pulses = wspdCtr;
  wspdCtr = 0;
  interrupts();
  
  if (timediff > 0) 
    wSpd = (float)pulses / timediff;
  else
    wSpd = 0;
}

bool isWindValid(float newValue, float oldValue) {
  if (isnan(newValue) || newValue < 0) return false;
  if (newValue > MAX_WIND_SPEED) return false;

  if (!avgInitialized) {
    windAvg = newValue;
    avgInitialized = true;
    return true;
  }
  float jump = fabs(newValue - oldValue);
  if (jump > MAX_JUMP) {
    return false;
  }
  
  if (newValue > 10) {
    if (newValue > windAvg * MAX_FACTOR) {
      return false;
    }
  }

  windAvg = windAvg + ALPHA * (newValue - windAvg);
  return true;
}

void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void mySleep() {
  transportSleep();
  adcOff();
  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
  sleep_enable();
  sleep_bod_disable();
  sleep_cpu();

  sleep_disable();
 }

 void wspdISR() {
  uint8_t now = tick12ms;
  if (now != prevTick) { // More than 12.5 ms since last tick, let's count up (12.5 ms = 80 m/s max)
    prevTick = now;
    wspdCtr++;
  }
}

void timer2Setup(void)
{ 
	TCCR2A = 0x00; 
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);
	TCNT2 = TIMER2_PRELOAD;
	TIMSK2 = (1<<TOIE2); // interrupt when TCNT2 is overflowed
}

ISR(TIMER2_OVF_vect) {
  tick12ms++;
  if (tick12ms >= TICKS_PER_SECOND) {
    tick12ms = 0;
    seconds++;
  }
  TCNT2 = TIMER2_PRELOAD;
}

void performReset() {
  cli();
  MCUSR = 0;
  wdt_enable(WDTO_15MS);
  while (true) { }
}
