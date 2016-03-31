//////////////////////////////////////////
// For the line trackers, the light and dark values must be tested experimentally

const int ci_Gripper_Line_Tracker = A0;
const int ci_Base_Line_Tracker = A1

//Calibration for Line Tracker Constants (Interval, Measuring and Tolerance)
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 100;

//Variables
unsigned int ui_Gripper_Line_Tracker_Data;
unsigned int ui_Base_Line_Tracker_Data;

//Input constants here, should be found experimentally
unsigned int ui_Gripper_Line_Tracker_Dark;
unsigned int ui_Gripper_Line_Tracker_Light;
unsigned int ui_Base_Line_Tracker_Dark;
unsigned int ui_Base_Line_Tracker_Light;

unsigned int ui_Line_Tracker_Tolerance;

int light_sensor_data = 0; //globabl variable to hold light sensor output data
int light_tolerance = 0; //might need to change this
int light_sensor_bright = 300;

pinMode(ci_Gripper_Line_Tracker, INPUT);
pinMode(ci_Base_Line_Tracker, INPUT);
ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  /////////////////////////////////////////////////////////////////////////////////////
  //THIS CODE SHOULD BE IMPLEMENTED WITH CHECKFORCUBES() FUNCTION
  while (cmFront > 25) //arbitrary distance
  {
    forward(100);
    void readLineTrackers();
    //Because it's the searching function, base line tracker data will be needed
    if (ui_Base_Line_Tracker_Data < (ui_Base_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
    {
      checkCube();
    }
  }
  void readLineTrackers() {
    ui_Gripper_Line_Tracker_Data = analogRead(ci_Gripper_Line_Tracker);
    ui_Base_Line_Tracker_Data = analogRead(ci_Base_Line_Tracker);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  //INDEXING CODE WITH THE LINE TRACKER SHOULD BE PLACED HERE
  //LOGIC FOR THIS CODE - WHEN THE LINE TRACKER SEES DARK TO LIGHT, -1 FROM 6
  //WHEN ENTERING THE INDEXING CODE, the arm will already be out, and the line tracker has detected
  //The Dark Value already
  void indexingCubes() {
    //May need some new globals for these variables
    int counter = 6;
    int dummyDark; 
    int raw2 = analogRead(1);
    long magneticFlux2 = raw2 - NOFIELD2;
    while (int counter != 1) {
      moveLeft(50);
      //Flux Distance should be tested, it is the distance when the hall effect is right
      //beside the tesseract while moving left
      if (magneticFlux2 == FluxDistance) {
        moveRight(20);
        // this delay value will be hardcoded, because the FluxDistance should always
        //be the same if flux is detected
        delay(20);
        stop_motors();
        grip_open(open_pos, closed_pos);
        //flick the wrist up a bit, then rotate
        servo_GripMotor.write(60); //may need to change this number
        rotateClockWise(20, 90);
        delay(20);
        stop_motors();
        servo_GripMotor.write(155); //back to flicked down position
      }
      else if (magneticFlux2 == 0) {
        moveLeft(50);
    
        if (ui_Gripper_Line_Tracker_Data == ui_Gripper_Line_Tracker_Dark) {
          moveLeft(50);
          dummyDark = 0;
          }
        if ((ui_Gripper_Line_Tracker_Data == !ui_Gripper_Line_Tracker_Dark)&&(dummyDark == 0)){
          counter--; 
        }
        if (counter == 2){
         //call a function that will rotate the robot into the corner)  
         lowIndexPlacing();
        }        
       /* if (counter == 1){
            stop_motors();
            grip_open(open_pos, closed_pos);
            servo_GripMotor.write(60); //may need to change this number
            rotateClockWise(20, 90);
            delay(20);
            stop_motors();
            servo_GripMotor.write(155); //back to flicked down position
          }*/
        }

      }
    }

    //////////////////////////////////////////////////
    //During Indexing, when the robot reaches the 2nd last tape, this function must be called
    void lowIndexPlacing(){
  

    }



  }
}




}
