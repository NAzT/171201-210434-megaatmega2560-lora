#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "CMMC_Interval.hpp"
#include <Adafruit_Sensor.h> 
#include <NMEAGPS.h>
#include <GPSport.h>

#include <rn2xx3.h>

//create an instance of the rn2483 library, using the given Serial port
rn2xx3 myLora(Serial2);


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

CMMC_Interval interval;

#include <CMMC_RX_Parser.h>
#include "packet.h"

void array_to_string(byte array[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}
CMMC_RX_Parser parser(&Serial3);

bool flag_dirty = false;

static CMMC_MASTER_PACKET_T master_packet;

void setup()
{
  Serial.begin(57600);
  gpsPort.begin(9600);
  Serial2.begin(57600);

  Serial.println("Initalizing LoRa module...");
  Serial.println("BEGIN...");
}

void loop()
{
  parser.process();
  GPSloop();
    interval.every_ms(2L * 1000, []() { 
      Serial.println(millis());
      Serial.print(gps_latitude); 
      Serial.print(","); 
      Serial.println(gps_longitude);
    });
}

