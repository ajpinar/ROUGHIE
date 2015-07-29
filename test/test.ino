#include <SoftwareSerial.h>
#include <UM7_BIN.h>
#include "RTClib.h"
#include <DynamixelSoftSerial.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>

/*
  Glider motion
  Command wirelesly the motion of the syringes and transfer the sensor data
  
  Connect SPI header of UM7 into Arduino (vin, gnd, rx tx)
  Connect the secundary header of UM7 into the GPS (rx2, tx2)
  
  Caution: SoftwareSerial has conflicts with servo
*/

#define GC_NULL  0
#define GC_RESET 1
#define GC_START 2
#define GC_STOP  3
#define GC_BEGIN 4

#define ME_TRANSCEIVE 0
#define ME_GLIDE_DOWN 1
#define ME_GLIDE_UP   2

#define SYNC_INTERVAL 500
uint32_t syncTime = 0;

UM7_BIN um7;
RTC_DS1307 rtc; // define the Real Time Clock object

File logfile;

struct param_t {
  int DWSensorBackPosition;       // bbak
  int DWSensorCenterPosition;     // bmid
  int DWSensorFrontPosition;      // bfro
  unsigned long int descentTime;  // desct
  unsigned long int riseTime;     // riset
  unsigned long int transTime;    // trant
  int pitch_pos_down;             // pdwn
  int pitch_pos_mid;              // pmid
  int pitch_pos_up;               // pupp
  int pitch_vel;                  // pvel
  int enPID;                      // epid
  
  float kp;             // pdwn
  float kd;              // pmid
  float ki;               // pupp
  float sp;                       // setp
  int mode;
} param;

const int PIN_solenoid = 2;    // Wire labeled with '0'
const int PIN_PS = 4;          // Wire labeled with '2'
const int PIN_pumpDir = 8;     // Wire labeled with '3'
const int PIN_pumpOnDIO = 13;  // Wire labeled with '5'
const int PIN_GO = 7;          // External switch to start go timer

const int PIN_dyna_rx = 11;      // Need a jumper on shield from pin 11 to RX
const int PIN_dyna_tx = 12;      // Need a jumper on shield from pin 12 to TX

SoftwareSerial ss(50, 51); // RX, TX

const int PIN_pressureSensorPin = A3;
const int PIN_voutPin = 3;
const int PIN_DWSensor = A2;

unsigned long int tSensor, dtSensor;
bool enSensor;

unsigned long int tPitch, dtPitch;
bool enPitch;

unsigned long int tGlider, dtGlider;
bool enGlider;

bool enDebug = 1;
bool SDgo = 0;
bool GCenable = 0;

char *help = "Commands: \n\tss [-e|-d] \n\tss [-s] \n\tgc [-e|-d] \n\tgc [-r] \n\tpr [-bbak|-bmid|-bfro|-desct|-riset|-trant|-pdwn|-pmid|-pupp|-pvel|-kp|-ki|-kd|-mode|-sp] [newValue]\n\tpr [-s]\n\tSDstart and SDstop\n\tGCenable and GCdisable\n";

const int NLOG = 256;
int nLog = 0;
struct dataLog_t {
  float pitch;
  float roll;
  int pressure;
  unsigned long int t;
} dataLog[NLOG];


void setup()
{
  // setup imu
  um7.begin(&Serial3);
  
  // setup communication and send the interface help message
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.println(help);
  delay(100);
  
  // setup synchronous events
  enSensor = 0;
  dtSensor = 500;
  tSensor = millis();

  enPitch = 1;
  dtPitch = 200;
  tPitch = millis();
  
  // initial parameters
  param.DWSensorBackPosition = 980;
  param.DWSensorCenterPosition = 825;
  param.DWSensorFrontPosition = 700;
  param.descentTime = 10000;
  param.riseTime = 10000;
  param.transTime = 5000;
  param.pitch_pos_down = 1;
  param.pitch_pos_mid = 250;
  param.pitch_pos_up = 550;
  param.pitch_vel = 200;
  param.enPID = 0;
  
  param.kp = 6.78965365633236;
  param.kd = 2.35342414552411;
  param.ki = 2.52672058212315;
  param.sp = 35.0;
  param.mode = 0;
  
  // setup glider and make it still
  gliderStateMachine(GC_BEGIN);
  gliderStateMachine(GC_STOP);
  
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
  }
  

  logfile.println("millis,stamp,datetime,Pressure,Pitch,Roll,DrawWire,Voltage,LinMassPos,tp1,tp2,yaw,rolld,pitchd,yawd,north,east,up");    

}

void loop()
{
  const int BUFF_LEN = 80;
  char buff[BUFF_LEN];
  DateTime now;
  
  gliderStateMachine(GC_NULL);

  int ret = um7.refresh(); // refresh navigational data
  
  uint32_t m = millis();
  
  if(SDgo == 1) { //NEW
  
  logfile.print(m);           // milliseconds since start
  logfile.print(", "); 
  
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

  // logging data 
  logfile.print(", ");    
  logfile.print(getFiltAnalog(PIN_pressureSensorPin));
  logfile.print(", ");    
  logfile.print(-um7.roll);// change the name if neccessary
  logfile.print(", ");
  logfile.print(-um7.pitch);
  logfile.print(", ");
  logfile.print(getFiltAnalog(PIN_DWSensor));
  logfile.print(", ");
  logfile.print(Dynamixel.readVoltage(1)/10.0);
  logfile.print(", ");
  logfile.print(Dynamixel.readPosition(1));
  logfile.print(", ");
  logfile.print(um7.t_p1);
  logfile.print(", ");
  logfile.print(um7.t_p2);
  logfile.print(", ");
  logfile.print(um7.yaw);
  logfile.print(", ");
  logfile.print(um7.rolld);
  logfile.print(", ");
  logfile.print(um7.pitchd);
  logfile.print(", ");
  logfile.print(um7.yawd);
  logfile.print(", ");
  logfile.print(um7.north);
  logfile.print(", ");
  logfile.print(um7.east);
  logfile.print(", ");
  logfile.print(um7.up);
  logfile.println("");
  
  
    // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) {return;}
  syncTime = millis();
  
  // updating FAT!
  logfile.flush();
  } //END NEW
  
//  while(1) {
//    //solOn();
//    //pump(-1);
//    
//    Serial.println("START");
//    pumpOut();
//      solOn();
//      pumpOn();
//      delay(5000);
//      pumpOff();
//      delay(3000);
//      //solOff();
//      //pumpIn();
//      delay(1000);
//      Serial.println("END");
//  }

    if(!GCenable) {
      gliderStateMachine(GC_STOP);
    }
  
  if(Serial.available()) // user command >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  {
    char arg[3][80];
    int len = Serial.readBytesUntil('\r', buff, BUFF_LEN);
    buff[len] = '\0';
    
    Serial.print("@");
    Serial.print(millis());
    Serial.print(" > ");
    Serial.println(buff); // echo back with timestamp
    
    sscanf(buff, "%s %s %s", arg[0], arg[1], arg[2]); // parsing

    if(strcmp(arg[0], "GCenable") == 0) {
      GCenable = 1;
      Serial.println("GC enabled");
    }
    if(strcmp(arg[0], "GCdisable") == 0) {
      Serial.println("GC disabled");
      GCenable = 0;
    }      
    
    if(strcmp(arg[0], "battery") == 0) {
      Serial.println(Dynamixel.readVoltage(1)/10.0);
    } 
    ///New
    if(strcmp(arg[0], "SDstart") == 0) {
      SDgo = 1;
    }
    if(strcmp(arg[0], "SDstop") == 0) {
      SDgo = 0;
    }
    ///END NEW
    if(strcmp(arg[0], "gc") == 0) {
      if(strcmp(arg[1], "-r") == 0) {
        gliderStateMachine(GC_RESET);
      }
      else if(strcmp(arg[1], "-e") == 0) {
        gliderStateMachine(GC_START);
      }
      else if(strcmp(arg[1], "-d") == 0) {
        gliderStateMachine(GC_STOP);
      }
      else {
        Serial.println(help);
        delay(100);
      }
      
    }
    
    
    if(strcmp(arg[0], "ss") == 0) {
      if(strcmp(arg[1], "-e") == 0) { // enable sensor broadcast
        enSensor = 1;
      }
      else {if(strcmp(arg[1], "-d") == 0) { // disable sensor broadcast
        enSensor = 0;
      }
      else {if(strcmp(arg[1], "-s") == 0) { // show logged data
        for(int n = 0; n < NLOG; n++) {
          Serial.print(dataLog[n].t); Serial.print(" "); Serial.print(dataLog[n].pitch); Serial.print(" "); Serial.println(dataLog[n].pressure);
        }
      }
      else{
        Serial.println(help); // print help is no match was found
        delay(100);
      }
    }}}
    else {if(strcmp(arg[0], "pr") == 0) { 
      if(strcmp(arg[1], "-s") == 0) {        // show all parameter values
        Serial.print(" bbak = "); Serial.println(param.DWSensorBackPosition);
        Serial.print(" bmid = "); Serial.println(param.DWSensorCenterPosition);
        Serial.print(" bfro = "); Serial.println(param.DWSensorFrontPosition);
        Serial.print(" desct = "); Serial.println(param.descentTime);
        Serial.print(" riset = "); Serial.println(param.riseTime);
        Serial.print(" trant = "); Serial.println(param.transTime);
        Serial.print(" pdwn = "); Serial.println(param.pitch_pos_down);
        Serial.print(" pmid = "); Serial.println(param.pitch_pos_mid);
        Serial.print(" pupp = "); Serial.println(param.pitch_pos_up);
        Serial.print(" pvel = "); Serial.println(param.pitch_vel);
        Serial.print(" epid = "); Serial.println(param.enPID);
        Serial.print(" kp = "); Serial.println(param.kp);
        Serial.print(" kd = "); Serial.println(param.kd);
        Serial.print(" ki = "); Serial.println(param.ki);
        Serial.print(" sp = "); Serial.println(param.sp);
        Serial.print(" mode = "); Serial.println(param.mode);
        Serial.println();
      }
      else {if(strcmp(arg[1], "-bbak") == 0) {        // update parameter: draw wire sensor
        param.DWSensorBackPosition = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-bmid") == 0) { // update parameter: descent time
        param.DWSensorCenterPosition = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-bfro") == 0) { // update parameter: rise time
        param.DWSensorFrontPosition = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-desct") == 0) { // update parameter: maximum pitch position
        param.descentTime = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-riset") == 0) { // update parameter: trimming pitch position
        param.riseTime = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-trant") == 0) { // update parameter: trimming pitch position
        param.transTime = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pdwn") == 0) { // update parameter: trimming pitch position
        param.pitch_pos_down = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pmid") == 0) { // update parameter: trimming pitch position
        param.pitch_pos_mid = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pupp") == 0) { // update parameter: trimming pitch position
        param.pitch_pos_up = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-pvel") == 0) { // update parameter: trimming pitch position
        param.pitch_vel = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-epid") == 0) { // update parameter: trimming pitch position
        param.enPID = atoi(arg[2]);
      }
      else {if(strcmp(arg[1], "-kp") == 0) {        // update parameter: draw wire sensor
        param.kp = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-kd") == 0) { // update parameter: descent time
        param.kd = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-ki") == 0) { // update parameter: rise time
        param.ki = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-sp") == 0) { // update parameter: rise time
        param.sp = atof(arg[2]);
      }
      else {if(strcmp(arg[1], "-mode") == 0) { // update parameter: rise time
        param.mode = atoi(arg[2]);
        pitchPID(0.0, 0, 0, 0); // reset pid states
        gliderStateMachine(GC_RESET);
      }
      else{
        Serial.println(help); // print help is no match was found
        delay(100);
      }
    }}}}}}}}}}}}}}}}}
    else{
      Serial.println(help); // print help is no match was found
      delay(100);
    }
  }} // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  
  if(enPitch & tPitch + dtPitch < millis()) { // 200 ms
    tPitch = millis();
    
    static int nPitch = 0;
    float pPos, pVel;
    
    switch(param.mode) {
      case 0: // nothing
      nPitch = 0;
      break;
      
      case 1: // square wave for system identification
      if(nPitch == 0)            Dynamixel.moveSpeed(1, param.pitch_pos_down, param.pitch_vel);
      if(nPitch == 10000/dtPitch) Dynamixel.moveSpeed(1,   param.pitch_pos_up, param.pitch_vel);
      
      nPitch++;
      if(nPitch >= 20000/dtPitch) nPitch = 0;
      break;
      
      case 2: // test PID parameters
      pitchPID(param.sp-(-um7.roll), &pPos, 0, dtPitch);
      Dynamixel.moveSpeed(1, int(pPos), param.pitch_vel);
      nPitch = 0;
      break;
    }
  }
   
  if(enSensor && tSensor + dtSensor < millis()) { // 500 ms
    tSensor = millis();
    
    dataLog[nLog].t = millis();
    dataLog[nLog].pressure = getFiltAnalog(PIN_pressureSensorPin); //analogRead(PIN_pressureSensorPin);
    dataLog[nLog].pitch = -um7.roll; // make sure it doesn't overflow
    dataLog[nLog].roll = -um7.pitch;
    Serial.print(dataLog[nLog].t); Serial.print(" "); Serial.print(dataLog[nLog].pitch); Serial.print(" "); Serial.println(dataLog[nLog].pressure); Serial.println(dataLog[nLog].roll);
        
    nLog++;
    if(nLog >= NLOG) nLog = 0;
    
//    if(ret >= 3) { // unexpected error detected
//      Serial.print("<"); Serial.print(ret); Serial.println(">"); 
//    }
//    
//    if(um7.updated_p) { // pose packet
//      um7.updated_p = 0;
//      Serial.print("t_p1 = "); Serial.print(um7.t_p1); Serial.print("; ");
//      Serial.print("roll = "); Serial.print(um7.roll); Serial.print("; ");
//      Serial.print("pitch = "); Serial.print(um7.pitch); Serial.print("; ");
//      Serial.print("yaw = "); Serial.print(um7.yaw); Serial.print("; ");
//      Serial.print("rolld = "); Serial.print(um7.rolld); Serial.print("; ");
//      Serial.print("pitchd = "); Serial.print(um7.pitchd); Serial.print("; ");
//      Serial.print("yawd = "); Serial.print(um7.yawd); Serial.print("; ");
//      Serial.print("north = "); Serial.print(um7.north); Serial.print("; ");
//      Serial.print("east = "); Serial.print(um7.east); Serial.print("; ");
//      Serial.print("up = "); Serial.print(um7.up); Serial.print("; ");
//      Serial.print("t_p2 = "); Serial.print(um7.t_p2); Serial.println("; ");
//    }
//    if(um7.updated_h) { // health packet
//      um7.updated_h = 0;
//      Serial.print("sats_in_view = "); Serial.print(um7.sats_in_view); Serial.print("; ");
//      Serial.print("sats_used = "); Serial.print(um7.sats_used); Serial.print("; ");
//      Serial.print("flags = "); Serial.print(um7.flags); Serial.print("; ");
//      Serial.print("HDOP = "); Serial.print(um7.HDOP); Serial.println("; ");
//    }
  }
  
}


void gliderStateMachine(int cmd) {
  
  static int state;        // machine state
  static bool entry;           // if it's first time executing the current state
  static unsigned long int t0; // initial time of current state
  static int cycleCount;
  
  const int dyna_id = 1;
  const long int dyna_Sbaud = 57600;
  
  float pPos, pVel;
  
  if(cmd == GC_BEGIN) { // execute once at beginning of test =================================
    if(enDebug)
      Serial.println(" glider.GC_BEGIN...");
      
    pinMode(PIN_solenoid, OUTPUT);
    pinMode(PIN_PS, OUTPUT);
    pinMode(PIN_pumpDir, OUTPUT);
    pinMode(PIN_pumpOnDIO, OUTPUT);
    pinMode(PIN_GO, INPUT);
  
    //analogReference(INTERNAL);
    
    Dynamixel.begin(dyna_Sbaud, PIN_dyna_rx, PIN_dyna_tx);
    Dynamixel.reset(dyna_id);
    
    psOn();
  }
  if(cmd == GC_RESET) { // reset to trimming position ==========================================
    if(enDebug)
      Serial.print(" glider.GC_RESET (please wait)...");
      
    Dynamixel.moveSpeed(dyna_id, param.pitch_pos_mid, param.pitch_vel);
    
    pump(1); // removes histeresis
    while(getFiltAnalog(PIN_DWSensor) < param.DWSensorCenterPosition);
    pump(-1);
    while(getFiltAnalog(PIN_DWSensor) > param.DWSensorCenterPosition);
    pump(0);
    
    enGlider = 0;
    
    if(enDebug)
      Serial.println(" done.");
  }
  
  if(cmd == GC_STOP) { // stop pump and stay =====================================================
    if(enDebug)
      //Serial.println(" glider.GC_STOP...");
    pump(0);
  }
  
  if(cmd == GC_START) { // begin gliding cycle ===================================================
    if(enDebug)
      Serial.println(" glider.GC_START...");
    cycleCount = 0;
    t0 = millis();
    nLog = 0;
    state = ME_GLIDE_DOWN;
    entry = 1;
  }
  
  if(cmd == GC_NULL) { // continue the normal machine state run ==================================
    if(GCenable) {
    switch(state) { // select current state
        
      case ME_GLIDE_DOWN: // permanent of gliding down ---------------------------------------------
        if(entry) { // entry
          if(enDebug)
            Serial.println(" glider.ME_GLIDE_DOWN...");
          Dynamixel.moveSpeed(dyna_id, param.pitch_pos_down, param.pitch_vel);
          pump(1);
          pitchPID(0.0, 0, 0, 0); // reset pid states
          entry = 0;
        }
        
        // during
        if(param.enPID) {
          pitchPID(-param.sp-(-um7.roll), &pPos, &pVel, dtGlider);
          Dynamixel.moveSpeed(dyna_id, (int) pPos, (int) pVel);
          Serial.print("roll = "); Serial.print(um7.roll); Serial.println("; ");
        }
        if(getFiltAnalog(PIN_DWSensor) > param.DWSensorBackPosition)
          pump(0);
        if(millis() > t0 + nLog*1000) {
          dataLog[nLog].pressure = analogRead(PIN_pressureSensorPin);
          dataLog[nLog].pitch = -um7.roll; // make sure it doesn't overflow
          nLog = nLog >= NLOG ? 0 : (nLog+1);
        }
        
        // exit condition
        if(millis() > t0 + (cycleCount+1)*param.descentTime + cycleCount*param.riseTime) { 
          entry = 1;
          state = ME_GLIDE_UP;
        }
        break;
        
      case ME_GLIDE_UP: // permanent of gliding down ---------------------------------------------------
        if(entry) { // entry
          if(enDebug)
            Serial.println(" glider.ME_GLIDE_UP...");
          Dynamixel.moveSpeed(dyna_id, param.pitch_pos_up, param.pitch_vel);
          pump(-1);
          pitchPID(0.0, 0, 0, 0); // reset pid states
          entry = 0;
        }
        
        // during
        if(param.enPID) {
          pitchPID(param.sp-(-um7.roll), &pPos, &pVel, dtGlider);
          Dynamixel.moveSpeed(dyna_id, (int) pPos, (int) pVel);
          Serial.print("roll = "); Serial.print(um7.roll); Serial.println("; ");
        }
        if(getFiltAnalog(PIN_DWSensor) < param.DWSensorFrontPosition)
          pump(0);
        if(millis() > t0 + nLog*1000) {
          dataLog[nLog].pressure = analogRead(PIN_pressureSensorPin);
          dataLog[nLog].pitch = -um7.roll; // make sure it doesn't overflow
          nLog = nLog >= NLOG ? 0 : (nLog+1);
        }
        
        // exit condition
        if(millis() > t0 + (cycleCount+1)*param.descentTime + (cycleCount+1)*param.riseTime) { 
          entry = 1;
          cycleCount++;
          if(cycleCount >= 3)            
            state = ME_TRANSCEIVE;
          else
            state = ME_GLIDE_DOWN;
        }
        break;
        
      case ME_TRANSCEIVE:
        if(entry) { // entry
          if(enDebug)
            Serial.println(" glider.ME_TRANSCEIVE...");
          Dynamixel.moveSpeed(dyna_id, param.pitch_pos_mid, param.pitch_vel);
          for(int n = 0; n < NLOG; n++) {
            Serial.print(dataLog[n].pitch); Serial.print(" ");
            dataLog[n].pitch = 0.0;
          }
          Serial.println();
          entry = 0;
        }
        
        // during
        if(getFiltAnalog(PIN_DWSensor) < param.DWSensorFrontPosition)
          pump(0);
        
        // exit condition
        if(millis() > t0 + 3*(param.descentTime+param.riseTime) + param.transTime) {
          t0 = millis();
          nLog = 0;
          cycleCount = 0;
          
          entry = 1;
          state = ME_GLIDE_DOWN;
        }
        break;
    }
    //Serial.print("("); Serial.print(pPos); Serial.print(", "); Serial.print(pVel); Serial.print(") ");
  }}
}

void pitchPID(float err, float* out, float* outd, int dt_ms) {
  
  static float err1 = 0.0;
  static float out1 = 0.0;
  static float sum = 0.0;

  const float dt = dt_ms * 1e-3;
  const float kp = param.kp;
  const float ki = param.ki;
  const float kd = param.kd;
  
  if(out == 0) { // if null pointer, reset internal states
    err1 = 0.0;
    out1 = 0.0;
    sum = 0.0;
    return;
  }
  
  *out = kp*err + ki*sum + kd*(err-err1)/dt + param.pitch_pos_mid;
  *out = constrain(*out, 1, 550); // saturation
  
  if(outd != 0) { // for soft movement
    *outd = abs((*out-out1)/dt);
    *outd = constrain(*outd, 0, 1000);
    //*outd = 200;
  }

  err1 = err; // last input value for derivative calculation
  out1 = *out;
  sum = sum + err*dt; // integral
}

void pump(int flow) {
  
  switch(flow) {
    case -1: // turn off and pump out
      pumpOff();
      delay(100);
      solOff();
      
      pumpOut();
      solOn();
      pumpOn();
      break;
      
    case 0: // just turn off
      pumpOff();
      delay(100);
      solOff();
      break;
      
    case 1: // turn off and pump in
      pumpOff();
      delay(100);
      solOff();
      
      pumpIn();
      solOn();
      pumpOn();
      break;
  }
}
  

// Equivalent to Serial.readBytesUntil(), except it trashes the buffer if timeout
// Use for chipkit or for softSerial
int ReadBytesUntil(char term, char* buff, int len)
{
  unsigned long int t0, timeout = 100;
  int n = 0;
  
  t0 = millis();
  while(millis() - t0 < timeout)
  {
    if(Serial.available())
    {
      buff[n] = Serial.read();
      if(buff[n] == term)
        return n;
      n++;
    }
  }
  return 0;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*
solOn(); Turns the solenoid on
 solOff(); Turns the solenoid off
 psOn(); Turns P5V on
 psOff(); Turns P5V off
 pumpIn(); 
 
 */
void solOn()
{
  // Turn solenoid on
  digitalWrite(PIN_solenoid, HIGH);
}
void solOff()
{
  // Turn solenoid off
  digitalWrite(PIN_solenoid, LOW);
}
void psOn()
{
  // Turn P5V on
  digitalWrite(PIN_PS, HIGH);
}
void psOff()
{
  // Turn P5V off
  digitalWrite(PIN_PS, LOW);
}
void pumpIn()
{
  // Set pump to pull water in
  digitalWrite(PIN_pumpDir, LOW);
}
void pumpOut()
{
  // Set pump to push water out
  digitalWrite(PIN_pumpDir, HIGH);
}
void pumpOn()
{
  digitalWrite(PIN_pumpOnDIO, HIGH); // THIS MUST BE UNCOMMENTED FOR THE PUMP TO WORK!!!!!!!!
//  digitalWrite(PIN_pumpOnDIO, LOW); // THIS MUST BE COMMENTED FOR THE PUMP TO WORK!!!!!!!!
}
void pumpOff()
{
  digitalWrite(PIN_pumpOnDIO, LOW);
}
int getFiltAnalog(int APIN)
{
  int val = 0;
  for(int a=0; a<10; a++) {
    val = val + analogRead(APIN);
  }
  val = val/10;
  return val;
}
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
}
