#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

static const uint32_t GPSBaud = 4800;
double lat,lng;
double speed;


// The TinyGPS++ object
TinyGPSPlus gps;


void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);
}

void loop()
{
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())){
      displayInfo(); 
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    lat = gps.location.lat();
    lng = gps.location.lng();
    Serial.print(lat, 6);
    Serial.print(F(","));
    Serial.print(lng, 6);
    Serial.println();
  }
  else
  {
    Serial.print(F("INVALID"));
    lat = 0;
    lng = 0;
  }

  Serial.print(F("Speed: "));
  if (gps.speed.isUpdated())
  {
    speed = gps.speed.mps();
    Serial.print(speed);
    Serial.println();  
  }
  else
  {
    Serial.print(F("INVALID"));
    speed = 0;
  }
  
  delay(500);

  Serial.println();
}







