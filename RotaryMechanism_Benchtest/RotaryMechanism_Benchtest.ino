#include <SoftwareSerial.h>
#include <DynamixelSoftSerial.h>
#include <Wire.h>

/*
  Glider motion
  Command wirelesly the motion of the syringes and transfer the sensor data
  
  Connect SPI header of UM7 into Arduino (vin, gnd, rx tx)
  Connect the secundary header of UM7 into the GPS (rx2, tx2)
  
  Caution: SoftwareSerial has conflicts with servo
*/

const int PIN_dyna_rx = 11;      // Need a jumper on shield from pin 11 to RX
const int PIN_dyna_tx = 12;      // Need a jumper on shield from pin 12 to TX

SoftwareSerial ss(50, 51); // RX, TX

char *help = "Commands: \n\thelp\t\t\tPrint this command table - example: type \"help\"\n\ttoggle_hold\t\tToggles servo holding torque on/off - example: type \"toggle_hold\"\n\tcurrent_position\tPrints the current position of the servo - example: type \"current_position\"\n\tparameters\t\tPrints the current parameters - example: type \"parameters\"\n\tbattery\t\t\tPrints the supply voltage - example: type \"battery\"\n\tspeed X\t\t\tSets the rotation speed to X - example: type \"speed 500\"\n\tturnto Y\t\tSets the desired turn angle to Y - example: type \"turnto 500\"\n\n\tBoth rotation angle and speed should be between 0-1023.\n\n";
char *details = "Startup notes:\n\tUpon connection with the Arduino, the servo will move to its center position\n\tand enough torque will be applied to the shaft to keep it stationary. After\n\tthe Arduino says \"Ready to go.\" it is ready for commands. Enter commands\n\taccording to the following definitions...\n\n";
char *bench_setup = "Bench setup notes:\n\n\tServo wires...\n\tWhite wire: goes to RS485 shield, screw terminal closest to outer edge.\n\tGreen wire, next to white: goes to RS485 shield, screw terminal closest to center of board.\n\tBlack wire: goes to +14V\n\tGreen wire, next to black: goes to GND\n\n";

int rotation_speed = 800; // 0 to 1023
int desiredRotationAngle = 512; // 0 to 1023

bool torque = 1;

const int dyna_id = 1;
const long int dyna_Sbaud = 57600;

void setup()
{
  delay(1000);
  
  // setup communication and send the interface help message
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.println(bench_setup);
  Serial.println(details);
  Serial.println(help);
  Serial.println("Setting up the servo...\n\n");
  
  Dynamixel.begin(dyna_Sbaud, PIN_dyna_rx, PIN_dyna_tx);
  Dynamixel.reset(dyna_id);
  delay(1000);
  
  if (abs(Dynamixel.readPosition(dyna_id) - desiredRotationAngle) > 10) {
    Serial.print("Resetting servo to ");
    Serial.print(desiredRotationAngle);
    Serial.println("...");
    Dynamixel.moveSpeed(dyna_id, desiredRotationAngle, rotation_speed);
    Serial.println("Done!");
  }
  
  Dynamixel.torqueStatus(dyna_id, ON);
  Serial.println("Servo set to hold shaft at current position.\n");
  
  Serial.println("Ready to go.\n");
  
}

void loop()
{
  const int BUFF_LEN = 80;
  char buff[BUFF_LEN];
  
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
    
    if(strcmp(arg[0], "battery") == 0) {
      Serial.println("Reading battery voltage...");
      Serial.print("Battery voltage: ");
      Serial.println(Dynamixel.readVoltage(dyna_id)/10.0);
    } 
    
    else if(strcmp(arg[0], "speed") == 0) {
      Serial.println("Changing rotation speed...");
      rotation_speed = atoi(arg[1]);
      if (rotation_speed < 0) {
        rotation_speed = 0;
      }
      else if (rotation_speed > 1023) {
        rotation_speed = 1023;
      }
      Serial.print("Rotation speed changed to: ");
      Serial.println(rotation_speed);
    }
    
    else if(strcmp(arg[0], "turnto") == 0) {
      Serial.println("Moving to desired turn angle...");
      desiredRotationAngle = atoi(arg[1]);
      if (desiredRotationAngle < 0) {
        desiredRotationAngle = 0;
      }
      else if (desiredRotationAngle > 1023) {
        desiredRotationAngle = 1023;
      }
      Dynamixel.moveSpeed(dyna_id, desiredRotationAngle, rotation_speed);
      while (Dynamixel.moving(dyna_id)) { };
      Serial.print("Moved to: ");
      Serial.println(Dynamixel.readPosition(dyna_id));
      torque = 1; // Dynamixel automatically keeps torque on shaft after it moves
    }
    
    else if(strcmp(arg[0], "parameters") == 0) {
      Serial.print("Speed: ");
      Serial.println(rotation_speed);
      Serial.print("Desired angle: ");
      Serial.println(desiredRotationAngle);
    }
    
    else if(strcmp(arg[0], "current_position") == 0) {
      Serial.print("Current Position: ");
      Serial.println(Dynamixel.readPosition(dyna_id));
    }
    
    else if(strcmp(arg[0], "toggle_hold") == 0) {
      torque = !torque;
      if (torque) {
        Dynamixel.torqueStatus(dyna_id, ON);
        Serial.println("Servo set to hold position.");
      }
      else if (!torque) {
        Dynamixel.torqueStatus(dyna_id, OFF);
        Serial.println("Servo set to spin freely.");
      }
        
    }
    
    else if(strcmp(arg[0], "help") == 0) {
      Serial.println(help);
    }
    else Serial.println("Invalid command. Try again, or type \"help\" for a list of valid commands.\n"); // print help is no match was found
  }
} // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

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
void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
}
