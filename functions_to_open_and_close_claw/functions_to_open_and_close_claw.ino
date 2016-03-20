#include "Servo.h"

Servo left_grip_servo; //two servo objects
Servo right_grip_servo;

//GLOBAL VARIABLES
int open_pos = 40; //angle associated with claw open
int closed_pos = 160; //angle associated with claw closed


void setup() {
  Serial.begin(9600);
  left_grip_servo.attach(4);
  right_grip_servo.attach(5);

}

void loop() {

 grip_close(open_pos, closed_pos);
delay(1000);

 grip_open(open_pos, closed_pos);

  delay(1000);
  
}


void grip_close(int open_pos, int closed_pos)
{
 for ( int n = open_pos; n < closed_pos; n++)
  {
    right_grip_servo.write(n);
    left_grip_servo.write(180-n); //180-n makes this one count move backwards
    delay(15);
    Serial.println(right_grip_servo.read());
  }
}

void grip_open(int open_pos, int closed_pos)
{
  for ( int n = closed_pos; n > open_pos; n--)
  {
    right_grip_servo.write(n);
    left_grip_servo.write(180-n); //180-n makes this one count move backwards
    delay(15);
      //Serial.println(n);
  }
}

