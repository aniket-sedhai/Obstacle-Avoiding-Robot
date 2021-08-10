#include "SimpleRSLK.h"
#include "Energia.h"

/* Defines bump switch functions of robot */
#include "Bump_Switch.h"
#include "QTRSensors.h"
#include "Romi_Motor_Power.h"
#include "Encoder.h"
#include "math.h"
#include "SPI.h"
#include "OneMsTaskTimer.h"
#include "LCD_SharpBoosterPack_SPI.h"
LCD_SharpBoosterPack_SPI myScreen(SHARP_128);
uint8_t myOrientation = 0;
uint16_t myCount = 0;
int number = 0;

#define LCD_VERTICAL_MAX    myScreen.getSize()
#define LCD_HORIZONTAL_MAX  myScreen.getSize()

//1.9027777778 encoder value is equivalent to 1 degree turn when the speed is approximately 15% of max speed
#define oneDegree 1.90277777778

  float remaining = 0;        //keeps track of the remaining straight distance after every collision
  float newDistance = 0;      //new distance after rerouting the path
  float AngleToTurnRad = 0;   //calculated angle in order to set the correct path to destination in radians
  int AngleToTurnDegree = 0;  //the same angle in degree
  int count = 0;              //to prevent the while loop from running twice


void turnRight(int angleinDeg)
{
    resetLeftEncoderCnt();
    enableMotor(BOTH_MOTORS);

   
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);

   
    int toturn = angleinDeg*oneDegree;

    
    while(getEncoderLeftCnt()<toturn) 
    {                         
       setMotorSpeed(LEFT_MOTOR, 16);
       setMotorSpeed(RIGHT_MOTOR, 15);
    }
    
    disableMotor(BOTH_MOTORS);
    enableMotor(BOTH_MOTORS);
    delay(100);
}


void turnLeft(int angleinDeg)
{
    resetLeftEncoderCnt();
    enableMotor(BOTH_MOTORS);
    
   
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);

   
    int toturn = angleinDeg*oneDegree;

  
    while(getEncoderLeftCnt()<toturn) 
    {                         
       setMotorSpeed(LEFT_MOTOR, 15);
       setMotorSpeed(RIGHT_MOTOR, 15);
    }
    
    disableMotor(BOTH_MOTORS);
    enableMotor(BOTH_MOTORS);
    delay(100);
}

void waitBtnPressed() {
  while(1) {
        if (digitalRead(LP_S1_PIN) == 0) break;
        if (digitalRead(LP_S2_PIN) == 0) break;
    digitalWrite(LP_RGB_LED_GREEN_PIN, HIGH);
        delay(500);
        digitalWrite(LP_RGB_LED_GREEN_PIN, LOW);
        delay(500);
  }
}


void manuever()
{
  digitalWrite(LP_RGB_LED_GREEN_PIN, HIGH);
  resetLeftEncoderCnt();
  enableMotor(BOTH_MOTORS);

  
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
  while(getEncoderLeftCnt()< 343)
  {
    setMotorSpeed(LEFT_MOTOR, 15);
    setMotorSpeed(RIGHT_MOTOR, 15);
  }
  
  resetLeftEncoderCnt();
  turnLeft(92);
  resetLeftEncoderCnt();

 
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
  while(getEncoderLeftCnt()< 343)
  {
    setMotorSpeed(LEFT_MOTOR, 15);  /* SPEED FOR RIGHT AND LEFT MOTOR ARE DIFFERENT TO OFFSET THE ERROR IN DIMENSIONS OF TIRES*/
    setMotorSpeed(RIGHT_MOTOR, 15);
  }
  
    resetLeftEncoderCnt();
    disableMotor(BOTH_MOTORS);
    digitalWrite(LP_RGB_LED_GREEN_PIN, LOW);
}


void drive()
{
  enableMotor(BOTH_MOTORS);
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);

  /*SPEED OF MOTORS ARE SET DIFFERENT TO OFFSET THE MISMATCHING DIMENSIONS OF TWO WHEELS*/
  setMotorSpeed(LEFT_MOTOR, 15);
  setMotorSpeed(RIGHT_MOTOR, 15);
  
}


void travel(float distance)
{
  bool reachedDestination = false;
  bool destinationArrived = false;
  if(getEncoderLeftCnt() < distance*343)
      {
        destinationArrived = false;
      }
      else
      destinationArrived = true;

  //CHECKING FOR THE ARRIVAL OF DESTINATION
  if (distance == 0 || destinationArrived || count == 1)
  {
    Serial.println("Destination arrived.");
    digitalWrite(LED_FL_PIN, HIGH);
    disableMotor(BOTH_MOTORS);
    return;
  }
  
  else
  {
    while((getEncoderLeftCnt() < distance*343) && count == 0) 
    {
      digitalWrite(LED_FR_PIN, HIGH);      
      drive();
      
      //CHECKING FOR COLLISION 
      //BUMP SWITCH NUMBER 2 HAS BEEN REMOVED FROM EFFECT TO CONSOLIDATE THE USE OF LCD
      
      for(int x = 0;x<6;x++)
      {
        if(isBumpSwitchPressed(x) == true) 
        {
          digitalWrite(LED_FR_PIN, LOW);
          digitalWrite(LED_BR_PIN, HIGH);
          int z = getEncoderLeftCnt();
          Serial.println("Collision detected");
          
          disableMotor(BOTH_MOTORS);        
          delay(100);
          enableMotor(BOTH_MOTORS);
          manuever();
          turnRight(90);
          resetLeftEncoderCnt();
          disableMotor(BOTH_MOTORS);        
          delay(100);
          enableMotor(BOTH_MOTORS);
          remaining = (distance+1) * 343 - z;
        
          AngleToTurnRad = atan(343.0/remaining);
          AngleToTurnDegree = AngleToTurnRad*57.2958;
                
          disableMotor(BOTH_MOTORS);      
          delay(100);
          
          enableMotor(BOTH_MOTORS);
          turnRight(AngleToTurnDegree);
          
          newDistance = sqrt(remaining*remaining + 343*343);
          
          travel(newDistance/343);
          count = 1;
          reachedDestination = true;
        }
        digitalWrite(LED_BR_PIN, LOW);
      }
      digitalWrite(LED_FR_PIN, HIGH);
      if (reachedDestination)
      destinationArrived = true;
    }
    disableMotor(BOTH_MOTORS);
  }
}




void setup()
{
  Serial.begin(115200);
  

    myScreen.begin();
    myScreen.clearBuffer();

    myScreen.setFont(1);
    myScreen.text(30, 20, "WELCOME!");

    myScreen.flush();
  

    setupRSLK();

  /*INITIALIZE LED PINS AS OUTPUTS*/
    pinMode(LED_FR_PIN, OUTPUT); 
    pinMode(LED_FL_PIN, OUTPUT); 
    pinMode(LED_BR_PIN, OUTPUT); 
    pinMode(LED_BL_PIN, OUTPUT);
    pinMode(LP_RED_LED_PIN, OUTPUT);
    pinMode(LP_RGB_LED_RED_PIN, OUTPUT);
    pinMode(LP_RGB_LED_BLUE_PIN, OUTPUT); 
    pinMode(LP_RGB_LED_GREEN_PIN, OUTPUT);

    /*INITIALIZE LaunchPad buttons as inputs*/
    pinMode(LP_S1_PIN, INPUT_PULLUP);
    pinMode(LP_S2_PIN, INPUT_PULLUP);

  
  delay(4000);
}



void loop()
{
  Serial.println("Waiting until left or right button is pushed");
  waitBtnPressed();
  travel(5.00);
}
