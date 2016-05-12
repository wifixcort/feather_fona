/*
  Feather FONA 808 Adafruit MQTT Datalogger

  Ricardo Mena C
  ricardo@crcibernetica.com
  http://crcibernetica.com

  License
  **********************************************************************************
  This program is free software; you can redistribute it 
  and/or modify it under the terms of the GNU General    
  Public License as published by the Free Software       
  Foundation; either version 3 of the License, or        
  (at your option) any later version.                    
                                                        
  This program is distributed in the hope that it will   
  be useful, but WITHOUT ANY WARRANTY; without even the  
  implied warranty of MERCHANTABILITY or FITNESS FOR A   
  PARTICULAR PURPOSE. See the GNU General Public        
  License for more details.                              
                                                        
  You should have received a copy of the GNU General    
  Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>.
                                                        
  Licence can be viewed at                               
  http://www.gnu.org/licenses/gpl-3.0.txt

  Please maintain this license information along with authorship
  and copyright notices in any redistribution of this code
  **********************************************************************************
  */

#include <SoftwareSerial.h>
#include <SPI.h>
#include <avr/wdt.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <LowPower.h>   //https://github.com/rocketscream/Low-Power
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//---------FONA network-------------
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define FONA_KEY A5
//-----------------------------------

//------------Network identifiers-----------------
uint8_t node_id = 15;   //This node id
//------------------------------------------------

uint8_t one_wire_bus = 12;

OneWire oneWire(one_wire_bus);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
uint8_t resolution = 3;

//-----------------FONA things--------------------
// this is a large buffer for replies
char replybuffer[255];
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

//------------------------------------------------

//-----Serial configurations-----
#define serial Serial

#define SERIAL_BAUD   115200
#define DEBUG //uncoment for debuging
#define FONA_BAUD 4800
//-------------------------------

// Adafruit IO configuration
#define AIO_SERVER           "io.adafruit.com"  // Adafruit IO server name.
#define AIO_SERVERPORT       1883  // Adafruit IO port.
#define AIO_USERNAME         "username"  // Adafruit IO username (see http://accounts.adafruit.com/).
#define AIO_KEY              "IO_KEY"  // Adafruit IO key (see settings page at: https://io.adafruit.com/settings).

#define MAX_TX_FAILURES 4  // Maximum number of publish failures in a row before resetting the whole sketch.

const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

uint8_t txFailures = 0;

// Feeds configuration
const char TEMPERATURE_FEED[] PROGMEM = AIO_USERNAME "/feeds/feather_temperature";
Adafruit_MQTT_Publish temperature_feed = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

const char BATTERY_FEED[] PROGMEM = AIO_USERNAME "/feeds/feather_battery";
Adafruit_MQTT_Publish battery_feed = Adafruit_MQTT_Publish(&mqtt, BATTERY_FEED);

uint8_t t_wait = 7;   //Wait T_WAIT*8 [8 because you sleep 8s], default 1min
uint8_t n_times = 0;  //Time to wait before send the packets

void init_fona(){
  fona_on();
  delay(2000);  
  fonaSS.begin(4800);
  
  if(!check_fona()){// See if the FONA is responding
    halt(F("Couldn't find FONA"));
  }//end if
  
  //APN configuration
  fona.setGPRSNetworkSettings(F(""), F(""), F(""));
#if defined(DEBUG)
  serial.println(F("Waiting 20s.."));
#endif
  
  delay(20000);//Wait for FONA

  gprs_disable();
  gprs_enable(0);

  mqtt_connect();

}//end init_fona

void mqtt_connect(){
  // Now make the MQTT connection.
  int8_t ret = mqtt.connect();
  if (ret != 0) {
#if defined(DEBUG)
    serial.println(mqtt.connectErrorString(ret));
#endif
    //init_fona();
    halt(F("FONA has some errors to initiate"));
  }else{
#if defined(DEBUG)
	serial.println(F("MQTT Connected!"));
#endif
  }//end if
  //  return 1;
}//end mqtt_connect

void setup() {
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, resolution);
  
  //---Serial init---
  serial.begin(SERIAL_BAUD);

  //-----------------
  //while(!Serial);
  serial.println("Start Program");
  
  pinMode(FONA_KEY, OUTPUT);
  digitalWrite(FONA_KEY, HIGH);

  t_wait  = t_wait*5;//Wait 5min to send

  delay(2000);  
}//end setup

void loop(){
  if(n_times >= t_wait){
    send_temperature();
    n_times = 0;//Back to start
  }else{//end if
    n_times++;//wait more
    serial.println("Go to sleep");
  }//end if
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  wdt_reset();
}//end loop

void send_temperature(){
  wdt_disable();
  serial.println("Starting Init FONA");
  init_fona();
  if((fona.GPRSstate()==0)||(fona.getNetworkStatus() != 1)||(!fona.TCPconnected()) || (txFailures >= MAX_TX_FAILURES)){
#if defined(DEBUG)
	serial.println(F("NetworkStatus or GPRS State errors"));
#endif
	wdt_disable();//20s for init_fona needed
	fona_off();
	delay(500);
	init_fona();//Reset FONA
	wdt_enable(WDTO_8S);
	wdt_reset(); 
  }
  for(uint8_t i = 0; i < 2; i++){//Send 2 messages
	uint32_t temperature = 0;
	sensors.requestTemperatures();//-->READ DS28B20 TEMPERATURA
	temperature= sensors.getTempCByIndex(0);
	log_temperature(temperature, temperature_feed);
  }//end for
  log_battery_percent(fona_get_battery(), battery_feed);
  fona_off();
  wdt_enable(WDTO_8S);//Reenable watchdog   
}//end secure_url_send

uint16_t fona_get_battery(void){
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end fona_get_battery

void halt(const __FlashStringHelper *error) {
  wdt_enable(WDTO_1S);
  wdt_reset();
#if defined(DEBUG)
  serial.println(error);
#endif
  while (1) {}
}//end halt

void log_temperature(uint32_t indicator, Adafruit_MQTT_Publish& publishFeed) {// Log battery
#if defined(DEBUG)
  serial.print(F("Publishing temperature: "));
  serial.println(indicator);
#endif
  if (!publishFeed.publish(indicator)) {
#if defined(DEBUG)
    serial.println(F("Publish failed!"));
#endif
    txFailures++;
  }else {
#if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
#endif
    txFailures = 0;
  }//end if
}//end log_battery_percent

void log_battery_percent(uint32_t indicator, Adafruit_MQTT_Publish& publishFeed) {// Log battery
#if defined(DEBUG)
  serial.print(F("Publishing battery percentage: "));
  serial.println(indicator);
#endif
  if (!publishFeed.publish(indicator)) {
#if defined(DEBUG)
    serial.println(F("Publish failed!"));
#endif
    txFailures++;
  }else {
#if defined(DEBUG)
    serial.println(F("Publish succeeded!"));
#endif
    txFailures = 0;
  }//end if
}//end log_battery_percent

void print_IMEI(void){
  // Print SIM card IMEI number.
#if defined(DEBUG)
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    serial.print("SIM card IMEI: "); serial.println(imei);
  }//end if  
#endif
}//end print_IMEI

int gprs_enable(int maxtry){
  // turn GPRS on
  wdt_enable(WDTO_2S);
  wdt_reset();
  if (!fona.enableGPRS(true)){
#if defined(DEBUG)
	serial.print(F("Failed to turn on GPRS = "));
	serial.println(maxtry);
#endif
    if(maxtry > 200){
      wdt_enable(WDTO_1S);
      wdt_reset();
      while(1){}
    }
    maxtry +=1;
    gprs_enable(maxtry);
  }else{
#if defined(DEBUG)
	serial.println(F("GPRS ON"));
#endif
  }
  wdt_reset();
  wdt_disable();
}//end gprs_enable

int gprs_disable(){
  // turn GPRS off
  if (!fona.enableGPRS(false)){
#if defined(DEBUG)
	serial.println(F("Failed to turn GPRS off"));
#endif
  }else{
#if defined(DEBUG)
	serial.println(F("GPRS OFF"));
#endif
    return 1;
  }//end if
}//end gprs_disable

void flushSerial() {
  serial.flush();
#if defined(DEBUG)
  serial.flush();
#endif
}//end flushSerial

int check_fona(){
  // See if the FONA is responding
  if (!fona.begin(fonaSS)) {           // can also try fona.begin(Serial1)
#if defined(DEBUG)
	serial.println(F("Couldn't find FONA"));
#endif  
    return 0;
  }//end if
#if defined(DEBUG)
  serial.println(F("FONA is OK"));
#endif
  return 1;  
}//end check_fona

void fona_on(){
#if defined(DEBUG)
  serial.println("Turning on Fona: ");
#endif
  /*while(digitalRead(FONA_PS)==LOW){//No PS in feather FONA
    digitalWrite(FONA_KEY, LOW);
	}*/
  digitalWrite(FONA_KEY, LOW);
  delay(4000);
  digitalWrite(FONA_KEY, HIGH);
}//end fona_on

void fona_off(){
#if defined(DEBUG)
  serial.println("Turning off Fona: ");
#endif
  /*while(digitalRead(FONA_PS)==HIGH){//No PS in feather FONA
    digitalWrite(FONA_KEY, LOW);
	}*/
  digitalWrite(FONA_KEY, LOW);
  delay(4000);
  digitalWrite(FONA_KEY, HIGH);
  
}//end fona_off

String float_to_string(float value, uint8_t places) {
  // this is used to cast digits 
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  //int i;
  float tempfloat = value;
  String float_obj = "";

  // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)  
  float d = 0.5;
  if (value < 0){
    d *= -1.0;
  }
  // divide by ten for each decimal place
  for (uint8_t i = 0; i < places; i++){
    d/= 10.0;
  }
  // this small addition, combined with truncation will round our values properly 
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

  if (value < 0){
    tempfloat *= -1.0;
  }
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }
  // write out the negative if needed
  if (value < 0){
    float_obj += "-";
  }//en if
  
  if (tenscount == 0){
    float_obj += String(0, DEC);
  }//en if
  
  for (uint8_t i = 0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    float_obj += String(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }//en for

  // if no places after decimal, stop now and return
  if (places <= 0){
    return float_obj;
  }//end if

  // otherwise, write the point and continue on
  float_obj += ".";

  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (uint8_t i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    float_obj += String(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }//end for
  return float_obj;
}
