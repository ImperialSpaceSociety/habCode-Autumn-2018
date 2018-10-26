/*
 * Created by Medad Newman on 24 oct 2018
 * 
 *THIS IS THE MOST IMPORTANT INFORMATION ON SETTING THE BAUDRATE RIGHT FOR THE GPS: https://ukhas.org.uk/guides:ublox8
 * 
 */

// In-built libraries
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// External libraries
#include <TinyGPS++.h>
#include <util/crc16.h>
#include <MS5611.h>

// Defining all the hard coded values
#define LEDPIN 13
#define RADIOPIN 8
#define GPSRXPIN 3
#define GPSTXPIN 2
#define CS_PIN 10
#define Ext_tmp_pin1 0
#define Ext_tmp_pin2 1
#define callsign "ICSEDS"


MS5611 sensor(&Wire);


char datastring[100];
char checksum_str[6];

String csvString;
String timeActual;
double int_temp = 0;
double ext_temp = 0;
double pressure = 0;
double posLat, posLongd, posAlt;
unsigned int CHECKSUM;
char data;
byte gps_set_sucess = 0;


// Initialise the telemetry count
int telem_counter = 3;

/*
 * ALL the initialisation stuff
*/
File file;
SoftwareSerial GPSSERIAL(GPSRXPIN, GPSTXPIN); // set up the serial connection with the GPS rx,tx
TinyGPSPlus gps; // create the GPS parser object

void setup() {
  // Pin related stuff
  pinMode(RADIOPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  // Serial for GPS and for the computer connection
  Serial.begin(38400);
  GPSSERIAL.begin(9600);


  // GPS STUFF/////////////////////////
  // START OUR SERIAL DEBUG PORT
  Serial.println(F("GPS Level Convertor Board Test Script"));
  Serial.println(F("03/06/2012 2E0UPU"));
  Serial.println(F("Initialising...."));
  //
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  //
  GPSSERIAL.print(F("$PUBX,41,1,0007,0003,4800,0*13\r\n")); 
  GPSSERIAL.begin(4800);
  GPSSERIAL.flush();
 
  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  Serial.println(F("Setting uBlox nav mode: "));
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;

  //SD card related setups
  initializeSD();
  createFile("telem.txt"); // Put a file unique for time
  closeFile();

}

void loop() {
    while (GPSSERIAL.available()){
    data = GPSSERIAL.read();
    //Serial.print("a"); // now this is due to the baud rate problem
    gps.encode(data);
  }
  
  read_lm35(); 
  read_ms5611();
  

  // Now process the  data and parse all the information we need
  posAlt = gps.altitude.meters();
  posLat = gps.location.lat();
  posLongd = gps.location.lng();
  timeActual = gps.time.value();
  
  // now make the telemetry string 
  // TODO: put in a counter and date and time
  csvString = "$$" callsign;
  csvString += ',' + String(timeActual);
  csvString += ',' + String(telem_counter);
  csvString += ',' + String(posLat, 6);
  csvString += ',' + String(posLongd,6);
  csvString += ',' + String(posAlt,6);
  csvString += ',' + String(int_temp,1); // one decimal places
  csvString += ',' + String(ext_temp,1); // one decimal places
  csvString += ',' + String(pressure,0); // zero decimal places

  //Serial.println(csvString);

  csvString.toCharArray(datastring,100); //why 140?
  
  // Standard code from the RTTY reference
  CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring, checksum_str);
  Serial.write(datastring);
  //rtty_txstring(datastring);


  // now log the whole string into a file in the sd card
  openFile("telem.txt");
  writeToFile(csvString);
  closeFile();

  
  // increase the telem by 1
  telem_counter +=1;
  delay(2000);
}



// read lm35
void read_lm35(){
  ext_temp = (analogRead(Ext_tmp_pin2)-analogRead(Ext_tmp_pin1))* (5.0*0.0001 * 1023.0);
  }

// read ms5611
void read_ms5611(){
  if (sensor.connect()>0) {
  //Serial.println(F("Error connecting..."));
  }
  sensor.ReadProm();
  sensor.Readout();
  int_temp = sensor.GetTemp()/100;
  pressure = sensor.GetPres();
}



// Below are all the SD card related code
void initializeSD()
{
  //Serial.println(F("Initializing SD card..."));
  pinMode(CS_PIN, OUTPUT);

  if (SD.begin())
  {
    Serial.println(F("SD card is ready to use."));
  } else
  {
    Serial.println(F("SD card initialization failed"));
    return;
  }
}

int createFile(char filename[])
{
  file = SD.open(filename, FILE_WRITE);

  if (file)
  {
    Serial.println(F("File created successfully."));
    return 1;
  } else
  {
    Serial.println(F("Error while creating file."));
    return 0;
  }
}

int writeToFile(String text)
{
  if (file)
  {
    file.println(text);
    Serial.println(F("Writing to file: "));
    Serial.println(text);
    return 1;
  } else
  {
    Serial.println(F("Couldn't write to file"));
    return 0;
  }
}

void closeFile()
{
  if (file)
  {
    file.close();
    Serial.println(F("File closed"));
  }
}

int openFile(char filename[])
{
  file = SD.open(filename);
  if (file)
  {
    Serial.println(F("File opened with success!"));
    return 1;
  } else
  {
    Serial.println(F("Error opening file..."));
    return 0;
  }
}


// All the functions below are RTTY related. 
void rtty_txstring(char * string)
{

  /* Simple function to sent a char at a time to
  ** rtty_txbyte function.
  ** NB Each char is one byte (8 Bits)
  */

  char c;

  c = *string++;

  while (c != '\0')
  {
    rtty_txbyte(c);
    c = *string++;
  }
}

void rtty_txbyte(char c)
{
  /* Simple function to sent each bit of a char to
  ** rtty_txbit function.
  ** NB The bits are sent Least Significant Bit first
  **
  ** All chars should be preceded with a 0 and
  ** proceded with a 1. 0 = Start bit; 1 = Stop bit
  **
  */

  int i;

  rtty_txbit(0); // Start bit

  // Send bits for for char LSB first

  for (i = 0; i < 7; i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1);

    else rtty_txbit(0);

    c = c >> 1;

  }
  rtty_txbit(1); // Stop bit
  rtty_txbit(1); // Stop bit
}

void rtty_txbit(int bit)
{
  if (bit)
  {
    // high
    digitalWrite(RADIOPIN, HIGH);
  }
  else
  {
    // low
    digitalWrite(RADIOPIN, LOW);
  }

  //                  delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
  delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
  // largest value that will produce an accurate delay is 16383
  // See : http://arduino.cc/en/Reference/DelayMicroseconds

}

uint16_t gps_CRC16_checksum(char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }

  return crc;
}


// code below has to do with putting the GPS in nav mode

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    GPSSERIAL.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  GPSSERIAL.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (GPSSERIAL.available()) {
      b = GPSSERIAL.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}

