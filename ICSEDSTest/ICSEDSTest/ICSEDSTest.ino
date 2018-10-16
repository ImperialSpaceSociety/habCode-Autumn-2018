/*
 Name:		ICSEDSTest.ino
 Created:	2/26/2017 1:35:24 PM
 Author:	robert
 
 Modified in early 2018 by Ignaty
 modified on 19 September 2018 by Medad Newman

 ref for radio/rtty: https://ukhas.org.uk/guides:linkingarduinotontx2
 IT is ABSOLUTELY critical to configure the GPS to work above 12 km this way: https://ava.upuaut.net/?p=738
*/
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <util/crc16.h>

#include <SD.h>
#include <SPI.h>
#include <Wire.h>


#define LEDPIN 13
#define RADIOPIN 4
#define GPSRXPIN 2
#define GPSTXPIN 3
char datastring[140];
char floatBuffer[10];
String csvString;
String timeActual;

/*
 * ALL the initialisation stuff
*/
int CS_PIN = 10;
String callsign = "NEMO";
File file;


/* Update this with the correct SLP for accurate altitude measurements */
SoftwareSerial GPSSERIAL(GPSRXPIN, GPSTXPIN); // set up the serial connection with the GPS rx,tx
TinyGPSPlus gps; // create the GPS parser object
double posLat, posLongd, posAlt;

// the setup function runs once when you press reset or power the board
void setup() {
  // Pin related stuff
  pinMode(RADIOPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  // Serial for GPS and for the computer connection
  Serial.begin(38400);
  GPSSERIAL.begin(9600);

  // SD card related setups
  initializeSD();
  createFile("telem.txt"); // Put a file unique for time
  closeFile();

}


// the loop function runs over and over again until power down or reset
void loop() {
  
  // The microcontroller listens to the gps for 2 seconds to receive a gps signal. This may be very dangerous because signals may arrive only after 2 seconds. Make it interrupt driven
  GPSSERIAL.begin(9600);
  for(int k = 0; k < 200; k++){ // why do this?? medad
    delay(10);
    while (GPSSERIAL.available())
    {
      //Serial.write(GPSSERIAL.read()); // for debugging only
      gps.encode(GPSSERIAL.read());
    }
  }
  GPSSERIAL.end();// not too sure if we need to end the serial connection

  // Now process the gps data and parse all the information we need
  posAlt = gps.altitude.meters();
  posLat = gps.location.lat();
  posLongd = gps.location.lng();
  timeActual = String(gps.time.hour()) + ':' + String(gps.time.minute()) + ':' + String(gps.time.second());
  
  // now make the telemetry string 
  // TODO: put in a counter and date and time
  csvString = "$$" + callsign + ',' + String(millis());// may not be a good idea to put the millis here
  csvString += ',' + timeActual;
  csvString += ',' + String(posLat, 6);
  csvString += ',' + String(posLongd,6);
  csvString += ',' + String(posAlt,6);
  csvString.toCharArray(datastring,140); //why 140?

  // Standard code from the RTTY reference
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);  // Calculates the checksum for this datastring
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring, checksum_str);
  Serial.write(datastring);
  rtty_txstring(datastring);


  // Now logging the accleration,gyro and magnetic data to CSV
  openFile("telem.txt");
  writeToFile(csvString);
  closeFile();
  

  Serial.println();
  // delay at the end of the loop. May not be needed
  delay(2000);
}


// Below are all the SD card related code
void initializeSD()
{
  Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);

  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
}

int createFile(char filename[])
{
  file = SD.open(filename, FILE_WRITE);

  if (file)
  {
    Serial.println("File created successfully.");
    return 1;
  } else
  {
    Serial.println("Error while creating file.");
    return 0;
  }
}

int writeToFile(String text)
{
  if (file)
  {
    file.println(text);
    Serial.println("Writing to file: ");
    Serial.println(text);
    return 1;
  } else
  {
    Serial.println("Couldn't write to file");
    return 0;
  }
}

void closeFile()
{
  if (file)
  {
    file.close();
    Serial.println("File closed");
  }
}

int openFile(char filename[])
{
  file = SD.open(filename);
  if (file)
  {
    Serial.println("File opened with success!");
    return 1;
  } else
  {
    Serial.println("Error opening file...");
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

