/*
  emonTH V1.4 Low Power DHT22 Humidity & Temperature & DS18B20 Temperature Node Example 

  Checkes at startup for presence of a DS18B20 temp sensor , DHT22 (temp + humidity) or both
  If it finds both sensors the temperature value will be taken from the DS18B20 (external) and DHT22 (internal) and humidity from DHT22
  If it finds only DS18B20 then no humidity value will be reported
  If it finds only a DHT22 then both temperature and humidity values will be obtained from this sesor
  
  Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson
  Builds upon JCW JeeLabs RF12 library, Arduino and Martin Harizanov's work

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- RFu JeeLib		https://github.com/openenergymonitor/RFu_jeelib   //library to work with CISECO RFu328 module
	- DHT22 Sensor Library  https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'
        - OneWire library	http://www.pjrc.com/teensy/td_libs_OneWire.html
	- DallasTemperature	http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip

  Recommended node ID allocation
  ------------------------------------------------------------------------------------------------------------
  -ID-	-Node Type- 
  0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes 
  5-10	- Energy monitoring nodes
  11-14	--Un-assigned --
  15-16	- Base Station & logging nodes
  17-30	- Environmental sensing nodes (temperature humidity etc.)
  31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
*/
// See block comment above for library info
#include <JeeLib.h>
#include <avr/power.h>
#include <avr/sleep.h>
//#include <RFu_JeeLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#ifndef __AVR_ATtiny84__
#include "DHT.h"
#endif


#define freq RF12_433MHZ                                              // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 18;                                                // EmonTH temperature RFM12B node ID - should be unique on network
const int networkGroup = 210;                                         // EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
                                                                      // DS18B20 resolution 9,10,11 or 12bit corresponding to (0.5, 0.25, 0.125, 0.0625 degrees C LSB), lower resolution means lower power

const int time_between_readings= 1;                                   // in minutes
const int TEMPERATURE_PRECISION=10;                                   // 9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to resplution of 0.5C, 0.25C, 0.125C and 0.0625C
#define ASYNC_DELAY 187                                               // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms

ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

volatile bool adcDone;

// for low-noise/-power ADC readouts, we'll use ADC completion interrupts
ISR(ADC_vect) { adcDone = true; }

// Hardwired emonTH pin allocations 
#if defined(__AVR_ATtiny84__)
const int DS18B20_PWR=3;
#define ONE_WIRE_BUS 9
#else
const int DS18B20_PWR=5;
const int DHT22_PWR=6;
const int LED=9;
const int BATT_ADC=1;
#define ONE_WIRE_BUS 19
#define DHTPIN 18
#endif

// Humidity code adapted from ladyada' example                        // emonTh DHT22 data pin
// Uncomment whatever type you're using!
#ifndef __AVR_ATtiny84__
// #define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
boolean DHT22_status;                                                 // create flag variable to store presence of DS18B20
#else
boolean DHT22_status = 0;
#endif

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
boolean DS18B20;                                                      // create flag variable to store presence of DS18B20 

typedef struct {                                                      // RFM12B RF payload datastructure
  	  int temp;
          int temp_external;
          int humidity;    
          int battery;          	                                      
} Payload;
Payload emonth;


boolean debug;
int numSensors; 
//addresses of sensors, MAX 4!!  
byte allAddress [4][8];                                              // 8 bytes per address

static byte vccRead (byte count =4) {
  set_sleep_mode(SLEEP_MODE_ADC);
  // use VCC as AREF and internal bandgap as input
#if defined(__AVR_ATtiny84__)
  ADMUX = 33;
#else
  ADMUX = bit(REFS0) | 14;
#endif
  bitSet(ADCSRA, ADIE);
  while (count-- > 0) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
  }
  bitClear(ADCSRA, ADIE);  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1024U) / (ADC + 1) - 50;
}

//################################################################################################################################
//################################################################################################################################
void setup() {
//################################################################################################################################
  
#if defined(__AVR_ATtiny84__)
  // get the pre-scaler into a known state
  cli();
  CLKPR = bit(CLKPCE);
  CLKPR = 0; // div 1, i.e. speed up to 8 MHz
  sei();

  // power up the radio on JMv3
  bitSet(DDRB, 0);
  bitClear(PORTB, 0);
#endif

#ifndef __AVR_ATtiny84__
  pinMode(LED,OUTPUT); digitalWrite(LED,HIGH);                       // Status LED on
#endif
   
  rf12_initialize(nodeID, freq, networkGroup);                       // Initialize RFM12B
  rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
  
  // Send RFM12B test sequence (for factory testing)
#ifndef __AVR_ATtiny84__
  for (int i=0; i<10; i++)                                           
  {
    emonth.temp=i; 
    //rf12_sendNow(0, &emonth, sizeof emonth);
    rf12_sendStart(0, &emonth, sizeof emonth);
    delay(100);
  }
  rf12_sendWait(2);
  emonth.temp=0;
#endif
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);

#if defined(__AVR_ATtiny84__)
  debug = 0;
#else
  if (Serial) debug = 1; else debug=0;                              //if serial UART to USB is connected show debug O/P. If not then disable serial
  
  if (debug==1)
  {
    Serial.begin(9600);
    Serial.println("emonTH"); 
    Serial.println("OpenEnergyMonitor.org");
    Serial.print("Node: "); 
    Serial.print(nodeID); 
    Serial.print(" Freq: "); 
    if (freq == RF12_433MHZ) Serial.print("433Mhz");
    if (freq == RF12_868MHZ) Serial.print("868Mhz");
    if (freq == RF12_915MHZ) Serial.print("915Mhz"); 
    Serial.print(" Network: "); 
    Serial.println(networkGroup);
    delay(100);
  }
#endif
  
#ifndef __AVR_ATtiny84__
  pinMode(DHT22_PWR,OUTPUT);
  pinMode(BATT_ADC, INPUT);
  digitalWrite(DHT22_PWR,LOW);
#endif
  pinMode(DS18B20_PWR,OUTPUT);
  digitalWrite(DS18B20_PWR,LOW);

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
#ifndef __AVR_ATtiny84__
  if (debug==0) power_usart0_disable();   //disable serial UART
  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for the DS18B20 library
  power_timer1_disable();
  power_spi_disable();
#endif

  //################################################################################################################################
  // Test for presence of DHT22
  //################################################################################################################################
#ifndef __AVR_ATtiny84__
  digitalWrite(DHT22_PWR,HIGH);
  dodelay(2000);                                                        // wait 2s for DH22 to warm up
  dht.begin();
  float h = dht.readHumidity();                                         // Read Humidity
  float t = dht.readTemperature();                                      // Read Temperature
  digitalWrite(DHT22_PWR,LOW);                                          // Power down
  
  if (isnan(t) || isnan(h))                                             // check if returns are valid, if they are NaN (not a number) then something went wrong!
  {
    if (debug==1) Serial.println(" - Unable to find DHT22 Sensor..trying agin"); delay(100);
    Sleepy::loseSomeTime(1500); 
    float h = dht.readHumidity();  float t = dht.readTemperature();
    if (isnan(t) || isnan(h))   
    {
      if (debug==1) Serial.println(" - Unable to find DHT22 Sensor for 2nd time..giving up"); 
      DHT22_status=0;
    } 
  } 
  else 
  {
    DHT22_status=1;
    if (debug==1) Serial.println("Detected DHT22 temp & humidity sesnor");  
  }
#endif

  //################################################################################################################################
  // Setup and for presence of DS18B20
  //################################################################################################################################
  digitalWrite(DS18B20_PWR, HIGH); delay(50); 
  sensors.begin();
  sensors.setWaitForConversion(false);                             //disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping http://harizanov.com/2013/07/optimizing-ds18b20-code-for-low-power-applications/ 
  numSensors=(sensors.getDeviceCount()); 
  
  byte j=0;                                        // search for one wire devices and
                                                   // copy to device address arrays.
  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  digitalWrite(DS18B20_PWR, LOW);
  pinMode(DS18B20_PWR,INPUT);
  
  if (numSensors==0)
  {
#ifndef __AVR_ATtiny84__
    if (debug==1) Serial.println("No DS18B20 detected");
#endif
    DS18B20=0; 
  } 
  else 
  {
    DS18B20=1; 
#ifndef __AVR_ATtiny84__
    if (debug==1) Serial.print("Detected "); Serial.print(numSensors); Serial.println(" DS18B20");
    if (DHT22_status==1) Serial.println("DS18B20 and DHT22 found, assuming DS18B20 is external sensor");
#endif
  }
  if (debug==1) delay(200);
  
  //################################################################################################################################
  
  // Serial.print(DS18B20); Serial.print(DHT22_status);
  // if (debug==1) delay(200);
   
  //digitalWrite(LED,LOW);
} // end of setup


//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{ 
  
  if ((DS18B20==0) && (DHT22_status==0))        //if neither DS18B20 or DHT22 is detected flash the LED then goto forever sleep
  {
#ifndef __AVR_ATtiny84__
    for (int i=0; i<20; i++)
    {
      digitalWrite(LED, HIGH); delay(200); digitalWrite(LED,LOW); delay(200);
    }
#endif
    cli();                                      //stop responding to interrupts 
    Sleepy::powerDown();                        //sleep forever
  }

  if (DS18B20==1)
  {
    pinMode(DS18B20_PWR,OUTPUT);
    digitalWrite(DS18B20_PWR, HIGH); dodelay(50); 
    for(int j=0;j<numSensors;j++) sensors.setResolution(allAddress[j], TEMPERATURE_PRECISION);      // and set the a to d conversion resolution of each.
    sensors.requestTemperatures();                                        // Send the command to get temperatures
    dodelay(ASYNC_DELAY); //Must wait for conversion, since we use ASYNC mode
    float temp=(sensors.getTempC(allAddress[0]));
    digitalWrite(DS18B20_PWR, LOW);
    pinMode(DS18B20_PWR,INPUT);
    if ((temp<125.0) && (temp>-40.0))
    {
      if (DHT22_status==0) emonth.temp=(temp*10);            // if DHT22 is not present assume DS18B20 is primary sensor (internal)
      if (DHT22_status==1) emonth.temp_external=(temp*10);   // if DHT22 is present assume DS18B20 is external sensor wired into terminal block
    }
  }
  
#ifndef __AVR_ATtiny84__
  if (DHT22_status==1)
  { 
    digitalWrite(DHT22_PWR,HIGH);                                                                                                  // Send the command to get temperatures
    dodelay(2000);                                             //sleep for 1.5 - 2's to allow sensor to warm up
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    emonth.humidity = ((dht.readHumidity())*10);

    float temp=(dht.readTemperature());
    if ((temp<85.0) && (temp>-40.0)) emonth.temp = (temp*10);

    digitalWrite(DHT22_PWR,LOW); 
  }
#endif
  
#ifndef __AVR_ATtiny84__
  emonth.battery=int(analogRead(BATT_ADC)*0.03225806);                    //read battery voltage, convert ADC to volts x10
#else
  emonth.battery=int(vccRead());
#endif
  
#ifndef __AVR_ATtiny84__
  if (debug==1) 
  {
    if (DS18B20)
    {
      Serial.print("DS18B20 Temperature: ");
      if (DHT22_status) Serial.print(emonth.temp_external/10.0); 
      if (!DHT22_status) Serial.print(emonth.temp/10.0);
      Serial.print("C, ");
    }
    
    if (DHT22_status)
    {
      Serial.print("DHT22 Temperature: ");
      Serial.print(emonth.temp/10.0); 
      Serial.print("C, DHT22 Humidity: ");
      Serial.print(emonth.humidity/10.0);
      Serial.print("%, ");
    }
    
    Serial.print("Battery voltage: ");  
    Serial.print(emonth.battery/10.0);
    Serial.println("V");
    delay(100);
  }
#endif

  
#ifndef __AVR_ATtiny84__
  power_spi_enable();  
#endif
  rf12_sleep(RF12_WAKEUP);
  //rf12_sendNow(0, &emonth, sizeof emonth);
  rf12_sendStart(0, &emonth, sizeof emonth);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
#if defined(__AVR_ATtiny84__)
  rf12_sendWait(3);
#else
  rf12_sendWait(2);
#endif
  rf12_sleep(RF12_SLEEP);
#ifndef __AVR_ATtiny84__
  power_spi_disable();  
  digitalWrite(LED,HIGH);
  dodelay(100);
  digitalWrite(LED,LOW);  
#endif
  
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;   
  Sleepy::loseSomeTime(time_between_readings*60*1000);  
  //Sleepy::loseSomeTime(2000);
  ADCSRA=oldADCSRA; // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
      
  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
      
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

