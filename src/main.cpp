#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/draw/lv_draw.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "autons.h"  // Include the header file for the autonomous routines

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

   // Pneumatic expressions
    pros::ADIDigitalOut mogo('B'); // Creates a pneumatic control on port B
    pros::ADIDigitalOut doinker('E'); // Creates a pneumatic control on port E
    pros::ADIDigitalOut hang('C'); // Creates a pneumatic control on port C

    // Toggles for pneumatics
    bool mogoActivated = false;
    bool doinkerActivated = false;


//drivetrain specifications

pros::MotorGroup left_motors({-3, -7, 13}, 
                        pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({12, 14, -2}, 
                        pros::MotorGearset::blue); // right motors use 600 RPM cartridges
pros::MotorGroup intakeMotors({19, -16}, 
                                        pros::MotorGearset::blue);

// create an imu on port 2
pros::Imu imu(2);

pros::Rotation vertical_encoder(-21); // creates a v5 rotation sensor for odom on port 1
// vertical tracking wheel
lemlib::TrackingWheel vertical(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.5);

//drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.5, // 12.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              470, // change to 470 or 480 if this causes issues somehow
                              -4.1 // horizontal drift is 2 (for now)
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // Increase P gain slightly to correct for drifting (10 -> 12)
    0, // Add a small I gain to compensate for constant drift
    24  ,  // Increase D gain for faster response to changes (4 -> 5)
    0,  // Anti windup
    4,  // Small error range, in inches
    100, // Small error range timeout, in milliseconds
    5,  // Large error range, in inches
    300, // Large error range timeout, in milliseconds
    100  // Maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI) 
                                              65 , // derivative gain (kD)
                                              1.5, // anti windup
                                              1, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

//sensors used in odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel 1, set to nullptr as we don't have any horizontal wheels
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have any horizontal wheels
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(10);
        }
    });
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
     int autonSelector = 1;

    switch (autonSelector) {
        case 1:
            redSideMogoRush();
            break;
        case 2:
            blueSideMogoRush();
            break;
        case 3:
            skillsAuton();
            break;
        default:
         
            break;
    }
    

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
/**
 * Runs in driver control
 */
void opcontrol() {
 
    while (true) {
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // Move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        // Intake control using right bumpers
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    intakeMotors.move_velocity(600); // Full speed forward for blue (600 RPM) cartridges
} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    intakeMotors.move_velocity(-600); // Full speed backward for blue (600 RPM) cartridges
} else {
    intakeMotors.move_velocity(0); // Stop intake motors when no bumper is pressed
}


        // Mogo control (toggle activation with X button)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            mogoActivated = !mogoActivated; // Toggle mogo state
            mogo.set_value(mogoActivated); // Activate or deactivate the pneumatic
        }

        // Doinker control (toggle activation with left bumper 1)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            doinkerActivated = !doinkerActivated; // Toggle doinker state
            doinker.set_value(doinkerActivated); // Activate or deactivate the pneumatic
        }

        // Hang control (activate with up arrow, deactivate with down arrow)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            hang.set_value(true); // Activate hang
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            hang.set_value(false); // Deactivate hang
        }

        // Delay to save resources
        pros::delay(10);
    }
}
