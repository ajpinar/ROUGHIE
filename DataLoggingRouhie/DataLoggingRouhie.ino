#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"

// how many milliseconds between grabbing data and logging it. 
#define LOG_INTERVAL  500 // mills between entries // shall we decrease the sampling time??

// how many milliseconds before writing the logged data permanently to disk

#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// The analog pins that connect to the sensors
#define PressurePin 0           // analog 0
#define ImuPin      1           // analog 1

RTC_DS1307 rtc; // define the Real Time Clock object

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  while(1);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("START");
  
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  //while(1);
  
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
//  if (!SD.begin(10, 11, 12, 13)) {
  if (!SD.begin(53, 51, 49, 47)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.csv";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();  
  if (!rtc.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  

  logfile.println("millis,stamp,datetime,RawReadingPressure,RawReadingIMU");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,RawReadingPressure,RawReadingIMU");
#endif //ECHO_TO_SERIAL
 
}

void loop(void)
{
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
#endif

  // fetch the time
  now = rtc.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  //logfile.print(',');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
  Serial.print(',');
#endif //ECHO_TO_SERIAL
 
  double RawReadingPressure = measPressure();
//  RawReadingIMU=-um7.roll; // From GUI' code
  int RawReadingIMU = 5;
  
// logging data 
  logfile.print(", ");    
  logfile.print(RawReadingPressure);
  logfile.print(", ");    
  logfile.print(RawReadingIMU);// change the name if neccessary
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(RawReadingPressure);
  Serial.print(", ");    
  Serial.print(RawReadingIMU);
#endif //ECHO_TO_SERIAL

  
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) {return;}
  syncTime = millis();
  
  // updating FAT!
  logfile.flush();

}
double measPressure(){
  double y = FilterAnalog(PressurePin);   // Range : 0..1023
  return y;
}
int FilterAnalog(int PIN)
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(PIN);
  }
  val = val/10;
  return val;
}



