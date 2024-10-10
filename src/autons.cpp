#include "main.h"   // Include this if you're using any PROS functionality or main definitions
#include "autons.h" // Include the header file where the functions are declared

// Define the red side Mogo Rush autonomous routine
void redSideMogoRush() {
    // Your code for the Red Side Mogo Rush
         // set position to x:0, y:0, heading:0
    chassis.setPose(-51.5, -63.5, 270);
   // turn to face heading 90 with a very long timeout
    // set position to x:0, y:0, heading:0
    
    // move 48" forwards
 chassis.moveToPoint(-14 ,-58, 3000, {.forwards = false}, true);
 chassis.turnToHeading(235, 600);

    
  // Full speed forward for intake motors
    chassis.moveToPoint(chassis.getPose().x + 5, chassis.getPose().y + 6, 3000, {.forwards = false}, true);
     
    // Activate the mogo mechanism (pneumatic control)
    mogo.set_value(true);  // Activate mogo mechanism (assuming 'true' activates it)
     intakeMotors.move_velocity(600);
 
  
}

// Define the blue side Mogo Rush autonomous routine
void blueSideMogoRush() {
    // Your code for the Blue Side Mogo Rush
    chassis.setPose(-51.5, -63.5, 270);
    chassis.moveToPoint(-10.5, -60.5, 3000, {.forwards = false}, true);
    chassis.turnToHeading(242, 600);

    intakeMotors.move_velocity(600);  // Spin intake motors
    mogo.set_value(true);  // Activate the mogo mechanism
    pros::delay(500);
}

// Define the skills autonomous routine
void skillsAuton() {
    // Your code for the Skills Auton routine
    chassis.setPose(-51.5, -63.5, 270);
    chassis.moveToPoint(24, 24, 3000, {.forwards = true}, true);
    chassis.turnToHeading(90, 600);

    intakeMotors.move_velocity(600);  // Spin intake motors
    mogo.set_value(true);  // Activate the mogo mechanism
    pros::delay(500);
}
