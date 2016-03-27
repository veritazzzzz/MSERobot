 void veerRight(int speedy, int xDistance){
    int slower = speedy - 50;
    leftDistance = xDistance + 5; // may need to change the +3
    // for veerLeft, the right side motors are operating faster than the left side
    // this is done until we get to our set x distance away + a small value (2-3cm)
    // this is used to drive straight and correct the drift for the omniwheels
    pingLeft();
    while (current_pos[0] < leftDistance) {
      servo_FrontLeftMotor.writeMicroseconds(1500 + speedy); // the difference in the veerRight and left 
      servo_FrontRightMotor.writeMicroseconds(1500 + slower); // is the speed of the wheels and the 
      servo_BackLeftMotor.writeMicroseconds(1500 + speedy); // distance of the left wall to the ultrasonic 
      servo_BackRightMotor.writeMicroseconds(1500 + slower);
    }
  }

   void veerLeft(int speedy, int xDistance){
    int slower = speedy - 50;
    leftDistance = xDistance + 3; // may need to change the +3
    // for veerLeft, the right side motors are operating faster than the left side
    // this is done until we get to our set x distance away + a small value (2-3cm)
    // this is used to drive straight and correct the drift for the omniwheels
    pingLeft();
    while (current_pos[0] < leftDistance) {
      servo_FrontLeftMotor.writeMicroseconds(1500 + slower);
      servo_FrontRightMotor.writeMicroseconds(1500 + speedy);
      servo_BackLeftMotor.writeMicroseconds(1500 + slower);
      servo_BackRightMotor.writeMicroseconds(1500 + speedy);
    }
  }
