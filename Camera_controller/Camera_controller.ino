// Created by Medad Newman on 26 October 2018

int ledPin = 13;      // LED connected to digital pin 9
int shutterPin = 2;      // LED connected to digital pin 9







void setup()
{
  pinMode(ledPin, OUTPUT);   // sets the pin as output
  pinMode(shutterPin, OUTPUT);   // sets the pin as output

  Serial.begin(9600);

  // Blink the LED rapidly to show it is starting up. Make sure the camera is not recording when it is blinking
  Serial.println("hefdgfdgre1");

  for (int counter=1; counter<10; counter = counter+1){
    digitalWrite(ledPin, HIGH);
    delay(300);
    digitalWrite(ledPin, LOW);
    delay(50);
  }
  delay(500);
  Serial.println("hereasdsadsfd1");
  
}

void loop()
{
  Serial.println("here1");
  digitalWrite(shutterPin,HIGH);  // Start the video
  delay(500);
  

  Serial.println("here2");
  digitalWrite(shutterPin,LOW);  //NOW wait a long time(5 minutes)
  delay(300000);             //300 seconds              



  Serial.println("here3");
  digitalWrite(shutterPin,HIGH);  // Now stop the video
  delay(500);

  Serial.println("here4");
  digitalWrite(shutterPin,LOW);  // Wait for the video to save
  delay(20000); // wiat for 20 seconds for the data to save
  
}
