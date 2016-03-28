delay( 20-ish seconds);

home pos = current pos;



void_loop(){
  
rotate_clockwise(90);
arm_function_1(); // moves it just above to check for cube without knocking it over

while(not found cube)  // ie is constantly checking hall effect
{
  move_left;  // length of 'loading zone' and then sweep back
  move_right;
}
if(cube found) // hall effect senses something
  {
   move to where mag flux is greatest; // move left and right until found
   
   lower arm and grab cube(); 
   
   rotate_counter_clockwise(90);
   
   while(current[1] > home_pos[1])
     move_backwards;
   
   move to 'nth' position; //will increment n each time cube found
   arm_function_2(); // places cube up into the frame
   
   go_home;  
  }


}


