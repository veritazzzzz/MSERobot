void pickUpCubeFloor() {
    //No need for arm servo because the gripper will do the flick of the wrist 
    //The vertical (downward) encoder position is 155, also the encoder for the arm should be at zero or angle a little bit?
    void grip_open(int open_pos, int closed_pos);
    for (int servo_GripMotorAngle = 70; servo_GripMotorAngle < 155; servo_GripMotorAngle++)
    {
      servo_GripMotor.write(servo_GripMotorAngle);
      delay(10);
    }
    void grip_close(int open_pos, int closed_pos);

    for (int servo_GripMotorAngle = 155; servo_GripMotorAngle > 70; servo_GripMotorAngle--)
    {
      servo_GripMotor.write(servo_GripMotorAngle);
      delay(10);
    }
  }


  void pickUpCubeRack() {
    //did not use the arm servo for this function b/c i assumed that we could keep the base arm
    //high enough so that we would only use the grip wrist encoder 
    void grip_open(int open_pos, int closed_pos);
    for (int servo_GripMotorAngle = 60; servo_GripMotorAngle < 70; servo_GripMotorAngle++)
      //not sure if 60 is the right number, can test this
      //also not sure if 70 is parallel to the rack or not 
    {
      servo_GripMotor.write(servo_GripMotorAngle);
      delay(10);
    }
    void grip_close(int open_pos, int closed_pos);

    for (int servo_GripMotorAngle = 155; servo_GripMotorAngle > 70; servo_GripMotorAngle--)
    {
      servo_GripMotor.write(servo_GripMotorAngle);
      delay(10);
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
