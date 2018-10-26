// Created by Medad Newman on 26 October 2018

int ledPin = 13;      // LED connected to digital pin 9
int shutterPin = 4;      // LED connected to digital pin 9

int val = 0;         // variable to store the read value


unsigned long seconds = 1000L; // !!! SEE THE CAPITAL "L" USED!!!
unsigned long minutes = seconds * 60;
unsigned long hours = minutes * 60; 


void setup()
{
  pinMode(ledPin, OUTPUT);   // sets the pin as output
  pinMode(shutterPin, OUTPUT);   // sets the pin as output

  Serial.begin(9600);

  // Blink the LED rapidly to show it is starting up. Make sure the camera is not recording when it is blinking
  Serial.write("hefdgfdgre1");

  for (int counter=1; counter<20; counter = counter+1){
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  delay(500);
    Serial.write("hereasdsadsfd1");
  
}

void loop()
{
  Serial.write("here1");
  digitalWrite(shutterPin,HIGH);  // Start the video
  delay(500);

  Serial.write("here2");
  digitalWrite(shutterPin,LOW);  //NOW wait a long time(5 minutes)
  delay(1000*60*3);

  Serial.write("here3");
  digitalWrite(shutterPin,HIGH);  // Now stop the video
  delay(500);

  Serial.write("here4");
  digitalWrite(shutterPin,LOW);  // Wait for the video to save
  delay(5000);
  
}
