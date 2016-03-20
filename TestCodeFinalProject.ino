#include <uSTimer2.h>
#include <I2CEncoder.h>
#include <CharliePlexM.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>

Servo servo_FrontRightMotor;
Servo servo_FrontLeftMotor;
Servo servo_BackLeftMotor;
Servo servo_BackRightMotor;

I2CEncoder encoder_FrontRightMotor;
I2CEncoder encoder_FrontLeftMotor;
I2CEncoder encoder_BackRightMotor;
I2CEncoder encoder_BackLeftMotor;
I2CEncoder encoder_GripMotor;

//PORT PIN CONSTANTS 
const int ci_Front_Right_Motor = 11;
const int ci_Front_Left_Motor = 8;
const int ci_Back_Left_Motor = 9;
const int ci_Back_Right_Motor = 10;

//EEPROM ADDRESSES
const int ci_Front_Left_Motor_Offset_Address_L = 12;
const int ci_Front_Left_Motor_Offset_Address_H = 13;
const int ci_Front_Right_Motor_Offset_Address_L = 14;
const int ci_Front_Right_Motor_Offset_Address_H = 15;

const int ci_Back_Left_Motor_Offset_Address_L = 16;
const int ci_Back_Left_Motor_Offset_Address_H = 17;       //not sure if 16-19 are correct 
const int ci_Back_Right_Motor_Offset_Address_L = 18;
const int ci_Back_Right_Motor_Offset_Address_H = 19;

//STOP SPEED FOR MOTORS - 1500 
const int ci_Front_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Front_Right_Motor_Stop = 1500;
const int ci_Back_Left_Motor_Stop = 1500;        
const int ci_Back_Right_Motor_Stop = 1500;

const int ci_Motor_Calibration_Time = 5000;

unsigned long ul_Front_Left_Motor_Position;
unsigned long ul_Front_Right_Motor_Position;
unsigned long ul_Back_Left_Motor_Position;
unsigned long ul_Back_Right_Motor_Position;

unsigned long ui_Front_Left_Motor_Offset; 
unsigned long ui_Front_Right_Motor_Offset;
unsigned long ui_Back_Left_Motor_Offset; 
unsigned long ui_Back_Right_Motor_Offset;


boolean bt_3_S_Time_Up = false;

unsigned long current_FrontRightPosition = 0;
unsigned long current_FrontLeftPosition = 0;
unsigned long current_BackLeftPosition = 0;
unsigned long current_BackRightPosition = 0;

void setup() {
   Serial.begin(9600);
   
 // Set up drive motors
  pinMode(ci_Front_Right_Motor, OUTPUT);
  servo_FrontRightMotor.attach(ci_Front_Right_Motor);
  pinMode(ci_Front_Left_Motor, OUTPUT);
  servo_FrontLeftMotor.attach(ci_Front_Left_Motor);
  pinMode(ci_Back_Left_Motor, OUTPUT);
  servo_BackRightMotor.attach(ci_Back_Right_Motor);
  pinMode(ci_Back_Right_Motor, OUTPUT);
  servo_BackLeftMotor.attach(ci_Back_Left_Motor);

  current_FrontRightPosition

  encoder_Front_Left_Motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Front_Left_Motor.setReversed(false);  // adjust for positive count when moving forward
  encoder_Front_Right_Motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Front_Right_Motor.setReversed(true);  // adjust for positive count when moving forward
  encoder_Back_Left_Motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Back_Left_Motor.setReversed(false);  // adjust for positive count when moving forward
  encoder_Back_Right_Motor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_Back_Right_Motor.setReversed(true);  // adjust for positive count when moving forward
  

}

void loop() {


 case 4: //Motor Calibration for Straightness 
 
 if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }
 if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_Front_Left_Motor.zero();
            encoder_Front_Right_Motor.zero();
            encoder_Back_Left_Motor.zero();
            encoder_Back_Right_Motor.zero();
            ul_Calibration_Time = millis();
            servo_FrontLeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_FrontRightMotor.writeMicroseconds(ui_Motors_Speed);
            servo_BackLeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_BackRightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_FrontLeftMotor.writeMicroseconds(ci_Front_Left_Motor_Stop);
            servo_FrontRightMotor.writeMicroseconds(ci_Front_Right_Motor_Stop);
            servo_BackLeftMotor.writeMicroseconds(ci_Back_Left_Motor_Stop);
            servo_Back_Front_Right_Motor.writeMicroseconds(ci_Back_Right_Motor_Stop);
            
            ul_Front_Left_Motor_Position = encoder_Front_Left_Motor.getRawPosition();
            ul_Front_Right_Motor_Position = encoder_Front_Right_Motor.getRawPosition();
            ul_Back_Left_Motor_Position = encoder_Back_Left_Motor.getRawPosition();
            ul_Back_Right_Motor_Position = encoder_Back_Right_Motor.getRawPosition();
            
            if (ul_Front_Left_Motor_Position > ul_Front_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Front_Right_Motor_Offset = (ul_Front_Left_Motor_Position - ul_Front_Right_Motor_Position) / 3.43;
              ui_Front_Left_Motor_Offset = 0;
            }
            if (ul_Front_Left_Motor_Position < ul_Front_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Front_Right_Motor_Offset = 0;
              ui_Front_Left_Motor_Offset = (ul_Front_Right_Motor_Position - ul_Front_Left_Motor_Position) / 3.43;
            }
            if (ul_Back_Left_Motor_Position > ul_Back_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Back_Right_Motor_Offset = 0;
              ui_Back_Left_Motor_Offset = (ul_Back_Right_Motor_Position - ul_Back_Left_Motor_Position) / 3.43;
            }
            if (ul_Back_Left_Motor_Position < ul_Back_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Back_Right_Motor_Offset = 0;
              ui_Back__Motor_Offset = (ul_Back_Motor_Position - ul_Back_Motor_Position) / 3.43;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Front Right = ");
            Serial.print(ui_Front_Right_Motor_Offset);
            Serial.print(", Front Left = ");
            Serial.println(ui_Front_Left_Motor_Offset);
            Serial.print(", Back Right = ");
            Serial.println(ui_Back_Right_Motor_Offset);
            Serial.print(", Back Left = ");
            Serial.println(ui_Back_Left_Motor_Offset);
#endif
            EEPROM.write(ci_Front_Right_Motor_Offset_Address_L, lowByte(ui_Front_Right_Motor_Offset));
            EEPROM.write(ci_Front_Right_Motor_Offset_Address_H, highByte(ui_Front_Right_Motor_Offset));
            EEPROM.write(ci_Front_Left_Motor_Offset_Address_L, lowByte(ui_Front_Left_Motor_Offset));
            EEPROM.write(ci_Front_Left_Motor_Offset_Address_H, highByte(ui_Front_Left_Motor_Offset));
            EEPROM.write(ci_Back_Right_Motor_Offset_Address_L, lowByte(ui_Back_Right_Motor_Offset));
            EEPROM.write(ci_Back_Right_Motor_Offset_Address_H, highByte(ui_Back_Right_Motor_Offset));
            EEPROM.write(ci_Back_Left_Motor_Offset_Address_L, lowByte(ui_Back_Left_Motor_Offset));
            EEPROM.write(ci_Back_Left_Motor_Offset_Address_H, highByte(ui_Back_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_ENCODERS
          Serial.print("Encoders FL: ");
          Serial.print(encoder_Front_Left_Motor.getRawPosition());
          Serial.print(", FR: ");
          Serial.println(encoder_Front_Right_Motor.getRawPosition());
          Serial.print(", BL: ");
          Serial.println(encoder_Back_Left_Motor.getRawPosition());
          Serial.print(", BR: ");
          Serial.println(encoder_Back_Right_Motor.getRawPosition());
#endif
         //Change depending on what case we are in 
          ui_Mode_Indicator_Index = 4;
        }
        break;
      }


  case 5: //change based on what case #
 
/*Mode 1:  
- Zero position where the hall effect detects 
- Ping Left, Back and Front where the left detects how far we are from the side wall 
- As the rear sensor increases and the front sensor decreases 
        - Ping left is held constant and we drive forward  
        If the hall

*/
}


//*****Driving Functions***** 
void forward(int speedy){
  servo_FrontLeftMotor.writeMicroseconds(1500+speedy); //forward
  servo_FrontRightMotor.writeMicroseconds(1500+speedy); //forward
  servo_BackLeftMotor.writeMicroseconds(1500+speedy); //forward
  servo_BackRightMotor.writeMicroseconds(1500+speedy); //forward 
}

void reverse(int speedy){
  servo_FrontLeftMotor.writeMicroseconds(1500-speedy); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500-speedy); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500-speedy); //reverse
  servo_BackRightMotor.writeMicroseconds(1500-speedy); //reverse  
}

void moveLeft(int speedy){
  servo_FrontLeftMotor.writeMicroseconds(1500-speed); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500+speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500+speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500-speed); //reverse 
}

void moveRight(int speedy){
  servo_FrontLeftMotor.writeMicroseconds(1500+speed); //forward
  servo_FrontRightMotor.writeMicroseconds(1500-speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500-speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500+speed); //forward 
}

void rotateClockwise(int speedy, int angle){
  // stop all motors 
  // record encoder positions
  // create "current positions" - This will be a function call so the # keeps increasing 
  // utilize the difference between current position and last known position 
  // all of this in a conditional while statement 
  // while (abs 1+2) < x degrees)
  //    {
  //      write servvos
  //     }
  // stop all motors 
  servo_FrontLeftMotor.writeMicroseconds(200); //forward
  servo_FrontRightMotor.writeMicroseconds(200); //reverse
  servo_BackLeftMotor.writeMicroseconds(200); //forward
  servo_BackRightMotor.writeMicroseconds(200);

  // record encoder positions 
  current_FrontRightPosition = encoder_FrontRightMotor.getRawPosition();
  encoder_FrontLeftMotor.getRawPosition();
  encoder_BackRightMotor.getRawPosition();
  encoder_BackRightMotor.getRawPosition();

  // create "current positions" - this will be a functi
  
  servo_FrontLeftMotor.writeMicroseconds(1500+speedy); //forward
  servo_FrontRightMotor.writeMicroseconds(1500-speedy); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500+speedy); //forward
  servo_BackRightMotor.writeMicroseconds(1500-speedy); //reverse  
  delay(angle); 
  
}
void rotateCounterClockwise(int speedy, int angle){
  //change and test numbers accordingly 
  servo_FrontLeftMotor.writeMicroseconds(1500-speedy); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500+speedy); //forward
  servo_BackLeftMotor.writeMicroseconds(1500-speedy); //reverse 
  servo_BackRightMotor.writeMicroseconds(1500+speedy); //forward 
  delay(angle);
}

void forwardLeftDiagonal(int speedy){
  servo_FrontRightMotor.writeMicroseconds(1500+speedy); //forward
  servo_BackLeftMotor.writeMicroseconds(1500+speedy); //forward
}

void forwardRightDiagonal (int speedy){
  servo_FrontLeftMotor.writeMicroseconds(1500+speedy); //forward
  servo_BackRightMotor.writeMicroseconds(1500+speedy); //forward
}

void reverseRightDiagonal(int speedy){
  servo_FrontRightMotor.writeMicroseconds(1500-speedy); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500-speedy); //reverse
}

void reverseLeftDiagonal(int speedy){
  servo_FrontLeftMotor.writeMicroseconds(1500-speedy); //reverse
  servo_BackRightMotor.writeMicroseconds(1500-speedy); //reverse 
}

void stop_motors(){
  servo_FrontLeftMotor.writeMicroseconds(200); 
  servo_FrontRightMotor.writeMicroseconds(200);
  servo_BackLeftMotor.writeMicroseconds(200);
  servo_BackRightMotor.writeMicroseconds(200); 
}

//*****Pinging Functions*****

//allow the user to ping the front ultrasonic sensor
void pingFront(int delayTime){
digitalWrite(ci_Front_Ultrasonic_Ping, LOW); //giving a short pulse before hand to ensure a clean high pulse 
delayMicroseconds(2);
digitalWrite(ci_Front_Ultrasonic_Ping, HIGH); //keep in mind name for ultrasonic sensor might be different for other people 
delayMicroseconds(delayTime); //used delay time so user could insert how long the ping is used for?? While it is driving along the left wall??
duration = pulseIn(ci_Front_Ultrasonic_Ping, HIGH);
cm = microsecondsToCentimeters(duration);
Serial.print(cm);
Serial.print("cm");
Serial.println();
}

void pingBack(){
digitalWrite(ci_Back_Ultrasonic_Ping, LOW); //giving a short pulse before hand to ensure a clean high pulse 
delayMicroseconds(2);
digitalWrite(ci_Back_Ultrasonic_Ping, HIGH); //keep in mind name for ultrasonic sensor might be different for other people 
delayMicroseconds(delayTime); //used delay time so user could insert how long the ping is used for?? While it is driving along the left wall??
duration = pulseIn(ci_Back_Ultrasonic_Ping, HIGH);
cm = microsecondsToCentimeters(duration);
Serial.print(cm);
Serial.print("cm");
Serial.println();
}

void pingRight(){
digitalWrite(ci_Right_Ultrasonic_Ping, LOW); //giving a short pulse before hand to ensure a clean high pulse 
delayMicroseconds(2);
digitalWrite(ci_Right_Ultrasonic_Ping, HIGH); //keep in mind name for ultrasonic sensor might be different for other people 
delayMicroseconds(delayTime); //used delay time so user could insert how long the ping is used for?? While it is driving along the left wall??
duration = pulseIn(ci_Right_Ultrasonic_Ping, HIGH);
cm = microsecondsToCentimeters(duration);
Serial.print(cm);
Serial.print("cm");
Serial.println();
}

void pingLeft(){
digitalWrite(ci_Left_Ultrasonic_Ping, LOW); //giving a short pulse before hand to ensure a clean high pulse 
delayMicroseconds(2);
digitalWrite(ci_Left_Ultrasonic_Ping, HIGH); //keep in mind name for ultrasonic sensor might be different for other people 
delayMicroseconds(delayTime); //used delay time so user could insert how long the ping is used for?? While it is driving along the left wall??
duration = pulseIn(ci_Left_Ultrasonic_Ping, HIGH);
cm = microsecondsToCentimeters(duration);
Serial.print(cm);
Serial.print("cm");
Serial.println();
}

//*****Encoder position functions"
void currentEncoderPosition(){
  
  
  }




// Code this stuff in a way where we have to input variables and it will be gucci. There will be some stuff that we can't, but in certain cases you can: 
// turning counterclockwise (allow the user to input an angle that it will turn to)
// driving straight/right/left 
// create arrays, code out the logistics of the X and Y (last known position/current position) 
// Pinging the back, left, right and front ultrasonic sensors
// Initialization 
// Mode 2, hard coding the indexing 
// Ultrasonic to see if there's any activity approaching and picking up the placed tesseract from the other robot 
// Flipping the Register and registering a negative Y position instead of positive 
// Somehow go home with the current position and going back to the last known position 

//Navigate - Moving and Mapping (have to communicate with each other) 
      //Moving can be subdivided into - void forward(


// Last Known Position - Stop from the proximity sensor 
// Current Position is the continous pinging of the ultrasonic sensors 

// Insert Functions Here  
    //Mode 1 Function, Mode 2 Function - other functions are put into these 
    
    //Turn Right/Left/Forward/Backward/RotateLeft/RotateRight
    //Extend and Retract Arms/Gripper  
    //DisposeFake  
    //GoHome 



    
    //Movement 

    //Mapping 
    // - Ping L, R, F, B 

  long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;
}

