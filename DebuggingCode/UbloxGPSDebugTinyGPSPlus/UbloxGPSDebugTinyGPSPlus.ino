#define GPSENABLE 5
#include <SoftwareSerial.h>
#include <TinyGPS++.h>


char data;
TinyGPSPlus gps;

SoftwareSerial gpsSerial(3,2);
 
void setup() {
 pinMode(GPSENABLE, OUTPUT);
 digitalWrite(GPSENABLE, HIGH);
  delay(2000);

 Serial.begin(38400);

 while(!Serial){
  
 }

 gpsSerial.begin(9600);
 Serial.print("!");

}
 
void loop() {
  /*if (gpsSerial.available()){
    Serial.write(gpsSerial.read());
  }*/

  while(gpsSerial.available()){
    
    data = gpsSerial.read();
     //delay(10);

    Serial.print("a");
    gps.encode(data);
  }
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
  delay(500);
}
