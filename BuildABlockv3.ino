#include <ArxRobot.h>                 // instantiated as ArxRobot at end of class header
#include <EEPROM.h>
#include <Wire.h>                     // I2C support
#include <Adafruit_PWMServoDriver.h>  // Adafruit Servo Shield Library
#include <Adafruit_VL53L0X.h>         // Adafruit Time of Flight Distance Sensor Library
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ArxRobot ArxRobot;       // instantiated as ArxRobot at end of class header

int8_t angle_Claw = 0;
int8_t angle_Vertical = 0;
int8_t angle_Horizontal = 0;
// Angles of the claw and arm to be transmitted to the 3DoT from the ArxRobot App

uint16_t last_Claw_State = 0;
uint16_t last_VertArm_State = 0;
uint16_t last_HoriArm_State = 0;
// Storing previous servo positions

uint16_t pulseLength1 = 0;
uint16_t pulseLength2 = 0;
uint16_t pulseLength3 = 0;
// Declaring Angle and Pulse Length Values

uint8_t ClawServo = 0;                
uint8_t vertArmServo = 1;
uint8_t horiArmServo = 2;
// Attaching Servos to the PCA9685 Servo Drive

#define SERVO_FREQ 50          // Analog servos run at ~50 Hz updates
#define SERVOMIN_Claw  2000    // minimum claw pulse length in micro seconds
#define SERVOMAX_Claw  2300    // maximum claw pulse length in micro seconds

#define SERVOMIN_LArm  1000    // minimum pulse length of vertical movement servo in micro seconds
#define SERVOMAX_LArm  2300    // maximum pulse length of vertical movement servo in micro seconds

#define SERVOMIN_RArm  1000    // minimum pulse length of horizontal movement servo in micro seconds
#define SERVOMAX_RArm  2075    // maximum pulse length of horizontal movement servo in micro seconds
// Defining Servos

const uint8_t CMD_LIST_SIZE = 3;   
// We are adding 3 commands (Claw_Move, Vertical_Move, Horizontal_Move)

#define Claw_Move         0x41  // Custom command for claw
#define Vertical_Move     0x42  // Custom command for vertical movement of arm
#define Horizontal_Move   0x43  // Custom command for horizontal movement of arm
// Defining Custom Command Addresses

bool ClawHandler(uint8_t cmd, int8_t param[], uint8_t n);       // Claw Movment
bool VerticalHandler(uint8_t cmd, int8_t param[], uint8_t n);   // Vertical Movement of Arm
bool HorizontalHandler(uint8_t cmd, int8_t param[], uint8_t n); // Horizontal Movement of Arm
// Custom Command Handlers

bool ClawHandler (uint8_t cmd, int8_t param[], uint8_t n)
{
  angle_Claw = param[0];
  pulseLength1 = map(angle_Claw, 0, 90, SERVOMIN_Claw, SERVOMAX_Claw);  // Maps angle of servo to pulse length in microseconds 
  if (last_Claw_State < pulseLength1)
  {
    for (uint16_t i = last_Claw_State; i < pulseLength1; i++) 
    {
    pwm.writeMicroseconds(ClawServo,i);
    }
    last_Claw_State = pulseLength1;
    delay(500);
  }
  else if (last_Claw_State > pulseLength1)
  {
    for (uint16_t i = last_Claw_State; i > pulseLength1; i--) 
    {
    pwm.writeMicroseconds(ClawServo,i);
    }
    last_Claw_State = pulseLength1;
    delay(500);
  }
  return false;
  }  
  // Handler for claw movement
  // Positions the servos in increments to the designated postition inputed by the user through the ArxRobot App

bool VerticalHandler (uint8_t cmd, int8_t param[], uint8_t n)
{
  angle_Vertical = param[0];
  pulseLength2 = map(angle_Vertical, 0, 90, SERVOMIN_LArm, SERVOMAX_LArm);  // Maps angle of servo to pulse length in microseconds 
  if (last_VertArm_State < pulseLength2)
  {
    for (uint16_t i = last_VertArm_State; i < pulseLength2; i++) 
    {
    pwm.writeMicroseconds(vertArmServo,i);
    }
    last_VertArm_State = pulseLength2;
    delay(500);
  }
  else if (last_VertArm_State > pulseLength2)
  {
    for (uint16_t i = last_VertArm_State; i > pulseLength2; i--) 
    {
    pwm.writeMicroseconds(vertArmServo,i);
    }
    last_VertArm_State = pulseLength2;
    delay(500);
  }
  return false;
}
  // Handler for the vertical movement of the arm
  // Positions the servos in increments to the designated postition inputed by the user through the ArxRobot App
  
bool HorizontalHandler (uint8_t cmd, int8_t param[], uint8_t n)
{
  angle_Horizontal = param[0];
  pulseLength3 = map(angle_Horizontal, 0, 90, SERVOMIN_RArm, SERVOMAX_RArm);  // Maps angle of servo to pulse length in microseconds 
  if (last_HoriArm_State < pulseLength3)
  {
    for (uint16_t i = last_HoriArm_State; i < pulseLength3; i++) 
    {
    pwm.writeMicroseconds(horiArmServo,i);
    }
    last_HoriArm_State = pulseLength3;
    delay(500);
  }
  else if (last_HoriArm_State > pulseLength3)
  {
    for (uint16_t i = last_HoriArm_State; i > pulseLength3; i--) 
    {
    pwm.writeMicroseconds(horiArmServo,i);
    }
    last_HoriArm_State = pulseLength3;
    delay(500);
  }
  return false;
}
  // Handler for horizontal movement of the arm
  // Positions the servos in increments to the designated postition inputed by the user through the ArxRobot App

ArxRobot::cmdFunc_t onCommand[CMD_LIST_SIZE] = {{Claw_Move,ClawHandler},{Vertical_Move,VerticalHandler},{Horizontal_Move,HorizontalHandler}};

void setup()
{
  Serial.begin(9600);               // default = 115200
  pwm.begin();
  ArxRobot.begin();
  ArxRobot.setOnCommand(onCommand, CMD_LIST_SIZE);
  ArxRobot.setCurrentLimit(60);     // 500mA Current Limit
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  last_Claw_State = SERVOMIN_Claw;
  last_VertArm_State = SERVOMIN_LArm;
  last_HoriArm_State = SERVOMIN_RArm;
  
  pwm.writeMicroseconds(ClawServo,SERVOMIN_Claw);
  pwm.writeMicroseconds(vertArmServo,SERVOMIN_LArm);
  pwm.writeMicroseconds(horiArmServo,SERVOMIN_RArm);
  // Start servos at their respective positions
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;
  ArxRobot.loop();
}
 
