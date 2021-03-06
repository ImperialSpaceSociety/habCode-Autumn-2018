#include <SPI.h>

#include <RFM98W_library.h>
RFMLib radio =RFMLib(PA4,PC15,255,255);
#define nss PA4
#define LORA_CTRL1 PA0
#define LORA_CTRL2 PA1 
void setup(){
  pinMode(PA4, OUTPUT);
  digitalWrite(PA4, HIGH);
  pinMode(LORA_CTRL1, OUTPUT);
  pinMode(LORA_CTRL2, OUTPUT);
  digitalWrite(LORA_CTRL1, LOW); //TXD
  digitalWrite(LORA_CTRL2, HIGH); //RXD
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);


  
  Serial.begin(38400);
  Serial.println("INIT");
  
  byte my_config[6] = {0x44,0x84,0x88,0xc0,0xfc, 0x08};
 
  radio.configure(my_config);
  radio.wRFM(0x06, 0x85); 
  delay(3000);
}

void loop(){
 // Serial.println(radio.rRFM(0x42),HEX);
 /*//
 // delay(50);
  if(radio.rfm_status ==0){
    Serial.println("BEGIN");
    RFMLib::Packet p;
    p.data[0]=255;
    p.data[1]=243;
    p.len = 2;
    radio.beginTX(p); 
    
    //while((digitalRead(PC15)!=1) && ((millis() - startTime) < 2000));
  //  radio.rfm_done = true;  

  
    attachInterrupt(PC15,RFMISR,RISING);

  }

  if(radio.rfm_done){
        Serial.println("Ending");   
    radio.endTX();
  }*/
    if(radio.rfm_status == 0){
    radio.beginRX(); 
    attachInterrupt(PC15,RFMISR,RISING);
  }

  if(radio.rfm_done){
        Serial.println("Ending");   
    RFMLib::Packet rx;
    radio.endRX(rx);
   for(byte i = 0;i<rx.len;i++){
     Serial.print(rx.data[i]);
     Serial.print("  ");
   }
   Serial.println();
   Serial.print("RSSI = ");
   Serial.print(rx.rssi);
   Serial.print(", SNR = ");
   Serial.println(rx.snr);
  }
  
}

void RFMISR(){
  //Serial.println("interrupt");
 radio.rfm_done = true; 
}


