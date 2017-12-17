#include <Arduino.h> 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
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

static int32_t bme_temperature = 0;
static int32_t bme_humidity = 0;


uint16_t seq = 0;

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
CMMC_Interval read_sensor; 
void led_on();
void led_off();
void initialize_radio();

#include <CMMC_RX_Parser.h> 
Adafruit_BME280 bme; // I2C 


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
  join_result = myLora.initABP("26041B8F", "C4732F8F77E37B4FD8D0A565C6771C70", "22D70570B3DA95B0BB335C6BB06FD067");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  // join_result = myLora.initOTAA("70B3D57ED00083FF", "62A53449E0B874D147FA871371FE23CF");

  myLora.setFrequencyPlan(TTN_US);

  Serial.println(myLora.sendRawCommand("radio set freq 925000000"));
  Serial.println(myLora.sendRawCommand("radio get freq"));

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?"); 
    delay(6000); //delay a minute before retry
    Serial.println("retrying..");
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
  Wire.begin();
  
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  bool status = bme.begin(0x76); 
  Serial.println(status);
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  } 

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
    read_sensor.every_ms(2L * 1000, []() { 
      float t = bme.readTemperature();
      float h = bme.readHumidity();

      if (isnan(h) || h == 0) {
          Serial.println("read bme280 failed... try again..");
      } 
      else {
        bme_humidity = h*100;
        bme_temperature= t*100;
        Serial.println(t);
        Serial.println(h); 
      }

    });

    interval.every_ms(4L * 1000, []() { 
      packet.type  = 1;
      packet.seq  = seq++;
      packet.field1 = gps_latitude;
      packet.field2 = gps_longitude;
      Serial.println(millis());
      Serial.print(gps_latitude/1000.0); 
      Serial.print(","); 
      Serial.println(gps_longitude/1000.0 ); 
      if (packet.field1 != 0) {
        Serial.print("sending... result: ");
        Serial.println(myLora.txCommand("mac tx uncnf 1 ", (uint8_t*)&packet, sizeof(packet))); 
        packet.type = 2;
        packet.field1 = bme_temperature;
        packet.field2 = bme_humidity;
        Serial.println(myLora.txCommand("mac tx uncnf 2 ", (uint8_t*)&packet, sizeof(packet))); 
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
