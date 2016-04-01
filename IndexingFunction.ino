  //INDEXING function*******
  //Assumes that it is already in the last array index
  //For this code I'm writing, I'm assuming the robot is facing the rack,
  // and that the arms are not in place
  void indexingFunction() {
    // call arm functions and input the respective numbers needed, robot arm is now up

    // while the gripper halleffect does nothing, move left slowly
    int gripperRaw = analogRead(1); //will need to change value
    // Make a global variable called "NOFIELD" and make it the value that the
    // hall effect detects when there is no field
    int gripperFlux = gripperRaw - NOFIELD;
    while (gripperFlux < 100) {
      // need to test the value (100) to get a set distance away from the next magnet
      // may need to change this as the hall effect might not be that strong
      moveLeft(10);
      //but if there is no flux the whole way through (no tesseracts, it has
      //to rely on the line tracker to limit it to only 5 tape lines passed
      // you will only need this once, for subsequent times, there will be a tesseract
      // and the hall effect will detect it
      if (readLineTracker() == 5)
      {
        moveLeft(10);
        delay(10); //delay by an arbitrary number so it can move slighty to the drop off zone
        stop_motors();
        // now insert the code where you place the tesseracts, so open the gripper and go back
        // if the robot has to enter this if statement, go back to the searchingFunction
        // and get out of this while loop
      }
    }
  }
