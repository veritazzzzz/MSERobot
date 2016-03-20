
//JULIAN'S VARIABLES
int current_pos[3]; //0th position is x coordinate, 1st is y, 2nd is directionality register
int last_known_cube_pos [3];
int home_pos[2]; //no directuionality b/c we know always pointing in the positive direction
//just x and y coordinates {[x][y]}

int cubesCollected; //keeps track of how many cubes have been collected
int numberOfPasses; //keeps track of how many times we've driven in the y-direction
// used to keep track of standard x-coordinate spacing
bool cubeFound;

/////////////////////////////////////////////
FOR NOW, LEAVING THIS AS HARD CODED IN
//////////////////////////////////////////////
int sideLength = 200; //value in cm's


void setup()
{
  cubesCollected = 0;
  numberOfPasses = 0;
  //initPos(); //initialze the home position only when the robot starts each round
  current_pos[2] = 0; //sets the direction to positive y orientation
  // TURN 180 FUNCTION SHOULD FLIP THIS VALUE TO 1
  //TO INDICATE TRAVELLING IN THE NEGATIVE DIRECTION

  cubeFound = false;

}

void loop()
{


}

//////////////////////////////////*
JULIAN ZANE
MARCH 19, 2016

//function initialzes the home position
//of the robot for future navigational use

*/////////////////////////////////
void initPos()
{
  //ping left and read 10 times to let values stabilize
  for (int i = 0; i < 10; i++)
  {
    home_pos[0] = pingLeft(); //sets x coordinate
    home_pos[1] = pingRight(); //sets y coordinate
  }
}


////////////////////////////////////*
JULIAN ZANE
MARCH 19, 2016

//function takes care of driving around
//and trying to find a cube
//DOES NOT CONCERN ITSELF IF THE CUBE
//IS REAL OR NOT JSUT YET (SEPERATE FUNCTION)
*////////////////////////////////////
void searchForCube()
{

  //while we're within our side of the course (our side of neutral zone)
  while (current_pos[0] < ((sideLength / 2) - 2))
  {

    //checks for wall in front of robot
    while (pingForward() > 25) //arbitrary distance
    {
      driveForward();
      //////////////////////////////////////////////////////////////////////
      //NOTE: MIGHT HAVE TO USE PINGFORWARD HALFWAY THROUGH OUR INCREASE OF Y COORDINATE
      //////////////////////////////////////////////////////////////////////

      //if travelling in positive y-direction
      if (current_pos[3] == 0)
      {
        current_pos[1] = pingBackward(); //update y coordinate (might not even need this)
        current_pos[0] = pingLeft(); //updates current x-coordinate

        //veerLeft() and veerRight() keep us driving relatively straight in y-direction
        //brings robot towards left wall if drifting right
        if ( current_pos[0] > (10 * numberOfPasses) //comparative value is standard robot width * number of passes
      {
        veerLeft(); //new function to steer slightly to the left
          //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER LEFT FOR A LITTLE AMOUNT
          //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
          //TO THIS PART OF THE CODE
        }

        //brings robot away from left wall if drifting left
        if ( current_pos[0] < (10 * numberOfPasses)
      {
        veerRight(); //new function to steer slightly to the left
          //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER RIGHT FOR A LITTLE AMOUNT
          //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
          //TO THIS PART OF THE CODE
        }
      }//end if(current_pos[3] == 0)



      ///////////////////////////////////////////////////////////////////
      //HAVE TO REWRITE ALL THE ABOVE CODE FOR WHEN WE TRAVEL IN NEGATIVE Y-DIRECTION
      ///////////////////////////////////////////////////////////////////


    //if travelling in negative y-direction
      if (current_pos[3] == 1)
      {
        current_pos[1] = pingBackward(); //update y coordinate (might not even need this)
        current_pos[0] = pingRight() + 20; //the added value accounts for width of robot

        //veerLeft() and veerRight() keep us driving relatively straight in y-direction
        //robot drifting left away from wall
        if ( current_pos[0] > (10 * numberOfPasses)
      {
        veerRight(); //new function to steer slightly to the left
          //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER LEFT FOR A LITTLE AMOUNT
          //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
          //TO THIS PART OF THE CODE
        }

        //brings drifting towards wall
        if ( current_pos[0] < (10 * numberOfPasses)
      {
        veerLeft(); //new function to steer slightly to the left
          //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER RIGHT FOR A LITTLE AMOUNT
          //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
          //TO THIS PART OF THE CODE
        }
      }//end if(current_pos[3] == 1)

    } //end while

    //if here, robot needs to turn around;
    turnClockwise(180);//function should have a case for 180, where it flips
                       //the directionality register in "current_pos"
                       //as well as crabbing left or right based on current_pos[2] (directionality)
                       //to set us up for the next y-direction pass
                       //and increment the num,berOfPasses variable by one

  }//end while

}//end function



//////////////////////////////////////////////////////////////////////////
//TO DEBUG, SERIAL PRINT THE ARRAY "current_pos" TO MAKE SURE ITS VALUES
//ARE BEING UPDATED IN REAL-TIME
//
//
//
/////////////////////////////////////////////////////////////////////////////





ADD ISR TO STOP ROBOT AND BUMP INTO FUNCTION TO CHECK AND PICK UP BLOCK




