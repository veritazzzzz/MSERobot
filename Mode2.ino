/////////////////////////////////////////////////////////////////
/////////////////////MODE 2 CODE/////////////////////////////////
  const int ci_Left_Microswitch = 1;    //will need to change these port # 
  const int ci_Right_Microswitch = 2; 
  
  pinMode(ci_Left_Microswitch, OUTPUT);
  pinMode(ci_Right_Microswitch, INPUT);


while (cmGrip > 10) //20 is arbitrary value, may be less, depending on how far the robot needs to get
{
  stop_motors();
}
delay(100);

indexingCubes2(); 
rotateClockwise(150,90); //Facing the rack now, will need to tweak the 90 degrees, won't be exact, must test 

while (digitalWrite(ci_Left_Microswitch,LOW) && digitalWrite(ci_Right_Microswitch,LOW)){
  forward(); 
}

if (digitalWrite(ci_Left_Microswitch,HIGH) && digitalWrite(ci_Right_Microswitch,HIGH)){
  forward();
  delay(10); // test the arbitrary value to drive forward
  stop_motors();
}

//PLACE CUBE TO PLATFORM - this code is the code from "Placing Cubes" that was pushed to GitHub 
//on a branch by Justin 
placeCubeOnTopRack();

//INSERT CLOSING ARM FUNCTIONS HERE 

rotateClockwise(150,180); //HARD CODED 

//THESE 3 CASES ALLOW THE ROBOT TO CORRECT ITSELF DEPENDING ON WHICH WAY IT IS SLANTED
//AND WHICH MICROSWITCH IS TRIGGERED FIRST 
while (digitalWrite(ci_Left_Microswitch,HIGH) && digitalWrite(ci_Right_Microswitch,HIGH)){
  forward(); 
}
while (digitalWrite(ci_Left_Microswitch,HIGH) && digitalWrite(ci_Right_Microswitch,LOW)){
  forward(); 
  //MUST TWEAK, POSSIBLE CRAB LEFT A SMALL DELAY, THIS IS TESTED 
}

while (digitalWrite(ci_Left_Microswitch,LOW) && digitalWrite(ci_Right_Microswitch,HIGH)){
  forward(); 
}
if (digitalWrite(ci_Left_Microswitch,LOW) && digitalWrite(ci_Right_Microswitch,LOW)){
  forward();
  delay(10); // test the arbitrary value to drive forward
  stop_motors();
  rotateClockwise(150,90); 
}


  // after this it loops to the top of the code again

  void pingGrip() {
  //Need to initialize for ci_Grip_Ultrasonic_Ping and Data)

  digitalWrite(ci_Grip_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);
  digitalWrite(ci_Grip_Ultrasonic_Ping, LOW);

  gripDuration = pulseIn(ci_Grip_Ultrasonic_Data, HIGH, 10000);
  cmGrip = microsecondsToCentimeters(gripDuration);

  Serial.print("Left distance = ");
  Serial.print(cmLeft);
  Serial.print("cm  ");
  //Serial.println();
}

void indexingCubes2() {

  int raw2 = analogRead(1);
  long magneticFlux2 = raw2 - NOFIELD2;
  grip_open(open_pos, closed_pos);
  //Flux Distance should be tested, it is the distance when the hall effect is right
  //beside the tesseract while moving left
  while (magneticFlux2 > 50) { //TEST THE VALUE OF 50, NOT SURE IF THIS IS THE GUCCI, PUT THE TESSERACT IN THE MIDDLE OF THE GRIPPER
    moveLeft(50);
    if (magneticFlux2 == fluxDistance) {
      //if the flux detected is the fluxDistance, the wrist grip servo should be raised so that
      // it does not hit the tesseract while moving left.

      ////////KEEP IN MIND THAT 70 IS THE GRIPPER ARM OUTWARD/////
      for (int servo_GripMotorAngle = 70; servo_GripMotorAngle > 60); servo_GripMotorAngle--) {
        servo_GripMotor.write(servo_GripMotorAngle);
        delay(10):
        }
        // move left a small hardcoded amount so that the open segment of the gripper can collect the cube
        moveleft(10);
        delay(10);

        for (int servo_GripMotorAngle = 60; servo_GripMotorAngle < 70); servo_GripMotorAngle++) {
      servo_GripMotor.write(servo_GripMotorAngle);
        delay(10):
        }
        grip_close(open_pos, closed_pos);
      }
    }
  }

  void placeCubeOnTopRack(){
      void grip_close(int open_pos, int closed_pos);
        for (int servo_GripMotorAngle = 70; servo_GripMotorAngle > 55 ; servo_GripMotorAngle--)
    {
      //this for loop tilts the arm a bit backwards (past the 90 degree mark)
      //a bit past 90 degrees so that the cube can be parallel with the rack
      // not sure if 55, we will need to test this 
      servo_GripMotor.write(servo_GripMotorAngle);
      delay(10);
    }
      for (int servo_TopArmMotorAngle = 0; servo_TopArmMotorAngle > 95; servo_TopArmMotorAngle++)
      { // remember to re-initialize pins for TopArmMotor      
        // not sure if 95, will have to test, similar to the grip motor, it will tilt
        // a bit more than the 90 degree mark  
        servo_TopArmMotor.write(servo_TopArmMotorAngle);
        delay(10);
      }
  }

