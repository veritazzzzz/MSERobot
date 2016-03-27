//Globals Needed for: rightDuration, rightDistance, cm (to debug) and delaytime
//may need to get rid of delay time and change it to a small #? 
// comment in the code 

void pingRight() {
  digitalWrite(ci_Right_Ultrasonic_Ping, LOW); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(2);
  digitalWrite(ci_Right_Ultrasonic_Ping, HIGH); //keep in mind name for ultrasonic sensor might be different for other people
  delayMicroseconds(delayTime); //used delay time so user could insert how long the ping is used for?? While it is driving along the left wall??
  rightDuration = pulseIn(ci_Right_Ultrasonic_Ping, HIGH);
  rightDistance = microsecondsToCentimeters(rightDuration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
}

//ci_Right_Ultrasonic_Ping is the constant integer pin number
//for the respective ultrasonic sensor 

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
