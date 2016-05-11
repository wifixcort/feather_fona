/*
  Feather FONA 808 Datalogger

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

#define IP_JSON "166.78.62.254/input/post.json?node="
#define IP_APIKEY "&apikey=d1699ac02ed979dd0c4af09b84a3c9f5&json="

//------------Network identifiers-----------------
uint8_t node_id = 15;   //This node id
//------------------------------------------------

uint8_t one_wire_bus = 12;

OneWire oneWire(one_wire_bus);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
uint8_t resolution = 3;

//---Paquetes to route----
String pck = "";//Packet to send
String msg = "";//Received packets
String stringEvent = "";//Temporal buffer to receive from RX/TX

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

uint8_t t_wait = 8;   //Wait T_WAIT*8 [8 because you sleep 8s], default 1min
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

  t_wait  = t_wait*5;//Wait 5min to send

  gprs_disable();
  gprs_enable(0);
 
}//end init_fona

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

  delay(2000);  
}//end setup

void loop(){
  if(n_times >= t_wait){
    send_temperature();
    n_times = 0;//Back to start
  }else{//end if
    n_times++;//wait more
  }//end if
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  wdt_reset();
}//end loop

void send_temperature(){
    wdt_disable();
    serial.println("Starting Init FONA");
    init_fona();
    for(uint8_t i = 0; i < 2; i++){//Send 2 messages
      msg += String(node_id);//add node ID
      msg += ' ';
      sensors.requestTemperatures();//-->READ DS28B20 TEMPERATURA
      msg += sensors.getTempCByIndex(0);
      msg += ' ';
      msg += String(fona_get_battery());//battery voltage in millivolts
      secure_url_send(msg);
      #if defined(DEBUG)
      serial.println(msg);
      #endif
      msg = "";//Clean on exit      
    }//end for
    fona_off();
    wdt_enable(WDTO_8S);//Reenable watchdog   
}//end send_temperature

uint16_t fona_get_battery(void){
  // Grab battery reading
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end fona_get_battery

void secure_url_send(String &url){
  if(send_url(url) == -1){
    #if defined(DEBUG)
    serial.println(F("Error sendind URL"));
    #endif
    if((fona.GPRSstate()==0)||(fona.getNetworkStatus() != 1)){
      #if defined(DEBUG)
        serial.println(F("NetworkStatus or GPRS State errors"));
      #endif        
      wdt_disable();//20s for init_fona needed
      delay(500);
      init_fona();//Reset FONA
      wdt_enable(WDTO_8S);
      wdt_reset(); 
    }
  }//end if
}//end secure_url_send

void halt(const __FlashStringHelper *error) {
  wdt_enable(WDTO_1S);
  wdt_reset();
  #if defined(DEBUG)
    serial.println(error);
  #endif
  while (1) {}
}//end halt

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

String json_split(String &message, String &node_id){
  String brak = "\{";
  String number = "";
  uint16_t j = 0;
  uint8_t data_num = 1;
  node_id = "";
  for(uint8_t i = 0; i < message.length(); i++){
    if((message[i] == ';')||(message[i] == ' ')){
      j = i+1;
      break;
    }else{
      node_id += message[i];
    }//end if
  }//end for

  for(j; j < message.length(); j++){
    if((message[j] == ';')||(message[j] == ' ')){
      brak += String(data_num) + "\:" + number + "\,";
      data_num++;
      //serial.println(brak);
      number = "";
    }else{
      if(message[j]!= '\r'){
        number += message[j];
      }
    }//end if
    
  }//end for
  brak += String(data_num) + "\:" + number+"\}";
  return brak;
  
}//end json_pck

// Post data to website
int send_url(String &raw_paq){
  flushSerial();
  uint16_t statuscode;
  int16_t length;
  String node = "";//Store node id
  String json = json_split(raw_paq, node);//split packet into json format and store node id througth reference
  String url = IP_JSON+node+IP_APIKEY;
  
  int data_len = json.length()+1;
  char data[data_len];
  json.toCharArray(data, data_len);
  #if defined(DEBUG)
    serial.println(json);
  #endif
  
  int l_url = url.length()+json.length();//strlen(data)
  char c_url[l_url];
  sprintf(c_url, "%s%s", url.c_str(),json.c_str());
  //flushSerial();
  
  #if defined(DEBUG)  
    serial.println(c_url);
    serial.println(F("****"));
  #endif
       
  if (!fona.HTTP_GET_start(c_url, &statuscode, (uint16_t *)&length)) {
    #if defined(DEBUG)
    serial.println("GPRS send failed!");
    #endif
    return -1;
  }else{
    #if defined(DEBUG)
    serial.println("GPRS send ok");
    #endif
    #if defined(FREERAM)
        serial.print("Free RAM TOP = ");
        serial.println(freeRam());
    #endif    
  }
  while (length > 0) {
    while (fona.available()) {
      char c = fona.read();     
    // Serial.write is too slow, we'll write directly to Serial register!
      #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
          loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
          UDR0 = c;
      #else
          #if defined(DEBUG)
          serial.write(c);
          #endif
      #endif
      length--;
      if (! length) break;
    }//end while
  }//end while
  #if defined(DEBUG)
    serial.println(F("\n****"));
  #endif
  fona.HTTP_GET_end();
}//end send_url

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
