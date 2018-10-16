/*
 * Created on 12 Oct 2018 by Medad Newman
 * This program is used to measure the temperature from the lm 35 sensor. 
 * Circuit taken from here:https://e2e.ti.com/support/sensors/f/1023/t/586616
 * Make sure you put the pins right for the lm35
 */

float vd = 0.6;
int analogPin1 = 1;     //  connected to analog pin 3
int analogPin2 = 2;     //  connected to analog pin 3
                       // outside leads to ground and +5V
int val1 = 0;           // variable to store the value read
int val2 = 0;           // variable to store the value read

void setup()
{
  Serial.begin(9600);              //  setup serial
}

void loop()
{
  float voltage1 = analogRead(analogPin1) * (5.0 / 1023.0);
  float voltage2 = analogRead(analogPin2) * (5.0 / 1023.0);

  float temp = -(voltage1-voltage2)/(0.01);
  Serial.println(temp);             // debug value
}
