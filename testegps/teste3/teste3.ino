#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;

SoftwareSerial ss(4,3);

void setup ()
{
Serial.begin(9600);
}

void loop ()
{

while (ss.available() > 0)
  gps.encode(ss.read());

if (gps.altitude.isUpdated())
  Serial.print(gps.altitude.meters());

if (gps.location.isUpdated()) {
Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
Serial.print("Date="); Serial.println(gps.date.value());
}
else
  Serial.print("Erro");
delay(800);
}
