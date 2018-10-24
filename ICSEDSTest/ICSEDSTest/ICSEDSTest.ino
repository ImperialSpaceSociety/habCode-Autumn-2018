/*
 Name:		ICSEDSTest.ino
 Created:	2/26/2017 1:35:24 PM
 Author:	robert
 
 Modified in early 2018 by Ignaty
 modified on 19 September 2018 by Medad Newman

 ref for radio/rtty: https://ukhas.org.uk/guides:linkingarduinotontx2
 IT is ABSOLUTELY critical to configure the GPS to work above 12 km this way: https://ava.upuaut.net/?p=738
 //TODO: get the analogue read for the sensors
 //TODO: get the library for reading the pressure sensor
 Senors to sample: 
 int temp
 ext temp
 pressure
 long
 lat 
 alt
 
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
#define GPSRXPIN 2
#define GPSTXPIN 3
#define CS_PIN 10
#define Ext_tmp_pin1 0
#define Ext_tmp_pin2 1
#define callsign "ICSEDS"


MS5611 sensor(&Wire);


char datastring[100];
char checksum_str[6];

String csvString;
String timeActual = "NONE";
double int_temp = 0;
double ext_temp = 0;
double pressure = 0;
double posLat, posLongd, posAlt;
unsigned int CHECKSUM;

// Initialise the telemetry count
int telem_counter = 3;

/*
 * ALL the initialisation stuff
*/
File file;
SoftwareSerial GPSSERIAL(GPSRXPIN, GPSTXPIN); // set up the serial connection with the GPS rx,tx
TinyGPSPlus gps; // create the GPS parser object

// the setup function runs once when you press reset or power the board
void setup() {
  // Pin related stuff
  pinMode(RADIOPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  // Serial for GPS and for the computer connection
  Serial.begin(38400);
  GPSSERIAL.begin(9600);

  // SD card related setups
  //initializeSD();

  //createFile("telemetry.txt"); // Put a file unique for time

  //closeFile();

}


// the loop function runs over and over again until power down or reset
void loop() {

  while (GPSSERIAL.available())
  {
    Serial.write(GPSSERIAL.read()); // for debugging only
    gps.encode(GPSSERIAL.read());
  }
  read_lm35(); 
  read_ms5611();
  

  // Now process the gps data and parse all the information we need
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
  csvString += ',' + String(int_temp,6);
  csvString += ',' + String(ext_temp,6);
  csvString += ',' + String(pressure,6);

  //Serial.println(csvString); 
    Serial.println(freeMemory());

  Serial.println(F("kjsf"));
      Serial.println(freeMemory());


  //csvString.toCharArray(datastring,100); //why 140?
  
  // Standard code from the RTTY reference
  CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring, checksum_str);
  Serial.write(datastring);
  rtty_txstring(datastring);

  // now log the whole string into a file in the sd card
  openFile("telemetry.txt");
  writeToFile(csvString);
  closeFile();

  // increase the telem by 1
  telem_counter +=1;
}

// read lm35
void read_lm35(){
  ext_temp = (analogRead(Ext_tmp_pin1)-analogRead(Ext_tmp_pin2))* (5.0*0.01 / 1023.0);
  }

// read ms5611
void read_ms5611(){
  if (sensor.connect()>0) {
  Serial.println(F("Error connecting..."));
  }
  sensor.ReadProm();
  sensor.Readout();
  int_temp = sensor.GetTemp();
  pressure = sensor.GetPres();
}


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
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

