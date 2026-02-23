/*  JSK - Vindmåler type batteri/solcelle!
    04-2022 - For bruk med Davis vindsensor og tenkt klasket opp på Liafjellet
    DS3231 for timing - sleep mode og timing funker dårlig på atmega328. I2C realtime klokke

    02-2023 - Endret rutiner for timing, optiboot for aktivering av WDT

    03-2023 - "Rensket" DS3231-kortet for alt annet enn selve chipen. Programmert BBSQW - SQW signal på batteri.
              Strømtrekk gikk fra 0.4 mA til ~0.5 uA, med strøm inn bare på batterikontakt.
              Interrupt 2x/sek fra SQW, sekund teller på høyt nivå. Tidligere lesing av klokka via I2C brukte
              ~ 2 ms per lesing, dyrebart!
              Passiv modus på MySensors bruker ca. 8 ms på å sende et flyttall, mot 60-80 ms i "aktiv" modus.

              Ved antatt snittforbruk på 1 mA med alt vil 5 Ah holde i 208 dager! Antagelig er snittforbuket lavere.

              Bruker egen rutine for sleep mode på MCU, MySensors sleep() vekker radioen hver gang
              sleep deaktiveres = hver interrupt = 2 - 62 ggr/sek.
              Radio vekkes nå kun når noe faktisk skal sendes.
    11-2025 - Byttet til DS1307. Fjernet Kalhamarfilter (unødvendig). Fjernet wdt_disable(), mulig årsak til frys
    12-2025 - Tester uten ekstern timer, bruker timer 2 på 80 Hz (12,5 ms per.) til timing. Litt mer strøm,
              men kan da filtrere bort vindmålerhjerteflimmer. Hvis vindm. trigger igjen før det er gått 1 timer2-tick
              blir den ignorert.
              Usikker på strømforbruk, men ser egentlig ut som det i snitt går litt mindre enn før (?). TWT
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
#define MY_RFM69_TX_POWER_DBM 13
#endif

/*
Liafjellet - id 6
*/
/*const unsigned int WIND_SPD_CALC_INTERVAL = 2; // Calc. wind speed every x second, filter crazy measurements
const unsigned int  SEND_SPD_INTERVAL = 6; // how often to send wind speed (sec)
const unsigned int  SEND_DIR_INTERVAL = 30; // wind dir sent here
const unsigned int  SEND_BATTERY_INTERVAL = 180; // how often to send battery data (sec)
const unsigned int  SEND_TEMP_INTERVAL = 120;
const float MAX_WIND_SPEED = 65.0;
const byte BATTERY_SENSE_PIN = A3;
const byte WDIR_SENSE = A0;
const byte WSPD_SENSE = 3;
const byte ONE_WIRE_BUS = A1;
const byte SQW_INT = 8;
const float R1  = 510;    // Vcc - BATTERY_SENSE_PIN
const float R2  = 77;    // BATTERY_SENSE_PIN - BATT_GND
const float Rratio = (R1 + R2) / R2;
const float battLow =  3.45;
const float battHigh = 4.20;
//#define MY_NODE_ID 6
//#define MY_RFM69_TX_POWER_DBM 17*/

/*
Liafjellet NY - id 6
*/
const unsigned int WIND_SPD_CALC_INTERVAL = 5; // Calc. wind speed every x second
const unsigned int  SEND_SPD_INTERVAL = 5; // how often to send wind speed (sec)
const unsigned int  SEND_DIR_INTERVAL = 30; // wind dir sent here
const unsigned int  SEND_BATTERY_INTERVAL = 1800; // how often to send battery data (sec)
const unsigned int  SEND_TEMP_INTERVAL = 120;
const byte BATTERY_SENSE_PIN = A3;
const byte WDIR_SENSE = A0;
const byte WSPD_SENSE = 3;
const byte ONE_WIRE_BUS = A1;
const byte SQW_INT = 8;
const float R1  = 569;    // Vcc - BATTERY_SENSE_PIN
const float R2  = 77;    // BATTERY_SENSE_PIN - BATT_GND
const float Rratio = (R1 + R2) / R2;
const float battLow =  2.7;
const float battHigh = 4.20;
#define MY_PASSIVE_NODE


/*
Brygga - id 8
*/
/*const unsigned int WIND_SPD_CALC_INTERVAL = 2; // Calc. wind speed every x second, filter crazy measurements
const unsigned int  SEND_SPD_INTERVAL = 6; // how often to send wind speed (sec)
const unsigned int  SEND_DIR_INTERVAL = 30; // wind dir sent here
const unsigned int  SEND_BATTERY_INTERVAL = 180; // how often to send battery data (sec)
const unsigned int  SEND_TEMP_INTERVAL = 120;
const float MAX_WIND_SPEED = 60.0;
const byte BATTERY_SENSE_PIN = A3;
const byte WDIR_SENSE = A2;
const byte WSPD_SENSE = 3;
const byte ONE_WIRE_BUS = A0;
const byte SQW_INT = 8;
const float R1  = 510;    // Vcc - BATTERY_SENSE_PIN
const float R2  = 81;    // BATTERY_SENSE_PIN - BATT_GND
const float Rratio = (R1 + R2) / R2;
const float battLow =  3.45;
const float battHigh = 4.20;
#define MY_NODE_ID 8
#define MY_RFM69_TX_POWER_DBM 10
*/

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

MyMessage msgDirection(CHILD_ID_DIRECTION, V_DIRECTION);
MyMessage msgSpeed(CHILD_ID_SPEED, V_WIND);
MyMessage msgGust(CHILD_ID_GUST, V_GUST);
MyMessage msgVoltage(CHILD_ID_VOLTAGE, V_VOLTAGE);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgPrefix1(CHILD_ID_SPEED, V_UNIT_PREFIX);  // Custom unit message.
MyMessage msgPrefix2(CHILD_ID_GUST, V_UNIT_PREFIX);  // Custom unit message.

const float MAX_WIND_SPEED = 65.0;      
const float MAX_JUMP = 8.0;             // maks tillatt hopp i m/s mellom to målinger
const float MAX_FACTOR = 2.8;           // ny måling kan maks være ~3x større enn gjennomsnitt
const float ALPHA = 0.15;               // for glidende gjennomsnitt

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
void(* resetFunc) (void) = 0;
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
  analogReference(INTERNAL);
  randomSeed(analogRead(A2));
  pinMode(WSPD_SENSE, INPUT);
  pinMode(WDIR_SENSE, INPUT);
  //pinMode(SQW_INT, INPUT_PULLUP);
  //pciSetup(SQW_INT); // Interrupt on sqw signal
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

void loop() {
  wdt_reset();

#ifdef TEST_MODE
  randWaitWspdTest = random(10, 20);
  if (tick12ms % randWaitWspdTest == 0) { // Random ms passed, simulating wspd. REMOVE BEFORE REAL USE!
    wspdISR();
  }
#endif
  
  uint32_t sec;
  noInterrupts();
  sec = seconds;
  interrupts();

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
      delay(20);
      uint16_t wdir_adc = analogRead(WDIR_SENSE);
      adcOff();

      if (wdir_adc > WDIR_ADC_MAX && wdir_adc < 900) WDIR_ADC_MAX = wdir_adc;
      wDir = map(wdir_adc, 0, WDIR_ADC_MAX, 0, 359);
      
      //Serial.println(analogRead(WDIR_SENSE));
      if (wDir > 359) wDir = 359;
      //else if (wDir < 0) wDir = 0;
        
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
  delay(20);
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
