#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "CMMC_Interval.hpp"
#include <Adafruit_Sensor.h> 
#include <NMEAGPS.h>
#include <GPSport.h> 
#include <rn2xx3.h>
#include "packet.h"

static NMEAGPS  gps; // This parses the GPS characters

static int32_t gps_latitude = 0;
static int32_t gps_longitude = 0;
static int32_t gps_altitude_cm = 0;
static uint32_t gps_us;

static void doSomeWork( const gps_fix & fix );
static void doSomeWork( const gps_fix & fix )
{
  if (fix.valid.location) {
    gps_latitude = fix.latitudeL();
    gps_longitude = fix.longitudeL();
    gps_altitude_cm = fix.altitude_cm();
    gps_us = fix.dateTime_us();
  }
} // doSomeWork
static void GPSloop();
static void GPSloop()
{
  while (gps.available( gpsPort ))
    doSomeWork( gps.read() );
} // GPSloop 

rn2xx3 myLora(Serial2);
CMMC_Interval interval; 
void led_on();
void led_off();
void initialize_radio();

#include <CMMC_RX_Parser.h> 
CMMC_RX_Parser parser(&Serial3); 
void initialize_radio()
{
  delay(100); //wait for the RN2xx3's startup message
  Serial1.flush();

  //print out the HWEUI so that we can register it via ttnctl
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the TTN UNO board.");
    delay(10000);
    hweui = myLora.hweui();
  }
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP("2604196F", "00B21288211391F29DE14E738385F16F", "5DBECD260FC1D6C71AF029121AAF0A7D");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  myLora.setFrequencyPlan(TTN_US);

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }

  Serial.println("Successfully joined TTN"); 
}

void setup()
{
  Serial.begin(57600);
  gpsPort.begin(9600);
  Serial2.begin(57600);

  Serial.println("Initalizing LoRa module...");
  Serial.println("BEGIN...");
  //output LED pin
  pinMode(13, OUTPUT);
  led_on(); 
  initialize_radio(); 
  // myLora.tx("TTN Mapper on TTN Uno node"); 
  led_off();
}
 
// uint8_t txBuffer[11];

CMMC_SENSOR_T packet;

void loop()
{
  parser.process();
  GPSloop();
    interval.every_ms(10L * 1000, []() { 
      packet.type  = 1;
      packet.seq1  = 1;
      packet.seq2  = 1;
      packet.field1 = gps_latitude;
      packet.field2 = gps_longitude;
      Serial.println(millis());
      Serial.print(gps_latitude/1000.0); 
      Serial.print(","); 
      Serial.println(gps_longitude/1000.0 ); 
      if (packet.field1 != 0) {
        Serial.print("sending... result: ");
        Serial.println(myLora.txCommand("mac tx uncnf 1 ", (uint8_t*)&packet, sizeof(packet))); 
      }
    });
}

void led_on()
{
  digitalWrite(13, 1);
}

void led_off()
{
  digitalWrite(13, 0);
}
