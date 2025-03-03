#include "main.h"
#include "EZ-Template/util.hpp"
#include "robodash/api.h"

//---------------------------------------------------Initialize important variables-----------------------------------------------------

// Piston variables
static bool clampPiston = false;   //---------------> toggle for mogo clamp
static bool doinkPiston = false;   //---------------> toggle for doinker
static bool intakePiston = false;  //---------------> toggle for intake piston
static bool doinkClamp = true;     //---------------> toggle for doinker clamp

// Color sort variables
static bool team = false;          //---------------> true = red    false = blue
static bool ringCol = false;
static bool isSorting = false;
static int blueLowerHue = 220;
static int blueHigherHue = 230;
static int redLowerHue = 350;
static int redHigherHue = 10;
static int distToSensor = 25;
static int timeToTop = 500;
static int flickDelay = 50;
static bool sortOverride = false;

// Arm variables
static int DunkPos = 0;            //---------------> arm macro integer
static bool isMoving = false;      //---------------> is arm active
static bool DONTFATFINGER = false; //---------------> helps to not fat finger at the beginning of a match
static float grabRingPos = 11;      //---------------> position values for the macro
static int returnRingGrabPos = 25;
static int scoreRingPos = 138;
static int returnHomePos = 5;
static int armTimeout = 0;         //---------------> timeout for arm being stuck

// Prog timer
static int MatchTimer = 0;

// Load images from sd
rd::Image logo("/usd/robotics/logo2.bin", "Logo 1");
rd::Image logo2("/usd/robotics/logo.bin", "Logo 2");
rd::Image Social15("/usd/robotics/+15.bin", "+15");
rd::Image Social1000("/usd/robotics/-100.bin", "+100");
rd::Image dhyan("/usd/robotics/dhyan.bin", "Dhyan");

//Initialize console
rd::Console console;

//Initialize auton selector
rd::Selector selector({
    {"BRush+", BlueLeftRush},
    {"RRush+", RedRightRush},
    {"BRushTug", BlueLeftRushTug},
    {"RRushTug", RedRightRushTug},
    {"QualB", QualBlueLeftRush},
    {"QualR", QualRedRightRush},
    {"Blue4-",sixRingBlue},
    {"Red4-", sixRingRed},
    {"BSWP-", BlueRightAWP},
    {"RSWP-", RedLeftAWP},
    {"Testing", test},
});

// Chassis constructor
ez::Drive chassis(
  {-10, -9, -8},  // Left Chassis Ports (negative port will reverse it!)
  {7, 6, 5},    // Right Chassis Ports (negative port will reverse it!)

  1, //-------------------------------> IMU Port
  3.25, //----------------------> Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
  450 //---------------------------------> Wheel RPM
);

// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(99, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(99, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

void initialize() {
  pros::screen::erase();
  pros::delay(500); //-----------------------> Stop the user from doing anything while legacy ports configure
  DunkPos = 0; //------------------------------------------> Arm stuff
  armTimeout = 0;
  ArmSensor.reset();
  ArmSensor.set_position(0);
  doinker_clamp.set_value(true); 
  intake_piston.set_value(false); //-----------------------> Sets intake piston to false so it starts down
  optical.set_integration_time(10);
  ArmSensor.set_data_rate(5);

  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_left_set(&vert_tracker);

  chassis.opcontrol_curve_default_set(2.1, 4.6);

  default_constants();

  chassis.initialize(); //---------------------------------> Initializes chassis and auton selector
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  pros::screen::erase();
  selector.focus();
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();                          // Resets PID targets to 0
  chassis.drive_imu_reset();                            // Reset gyro position to 0
  chassis.drive_sensor_reset();                         // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);// Set motors to hold. This helps autonomous consistency
  ArmSensor.reset();                                    // Rotation sensor reset and set to 0 for consistency
  ArmSensor.set_position(0);

  pros::Task console_display([&]() { //-------------------------> printing important shit to the brain
    while (true) {
      
      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) 
      && master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        opcontrol();
      }

      std::string ringStr = "";
      std::string autoStr = "";

      // Prints the ring color
      if (ringCol)
        ringStr = "Red";
      else if(!ringCol)
        ringStr = "Blue";

      if(selector.get_auton().has_value() == false){
        autoStr = "NOTHING   ";
      }else{
        autoStr = selector.get_auton()->name;
      }

      console.printf("X_COORD: %.3f \n",chassis.odom_x_get()); //------------> x, y, and heading values are printed
      console.printf("Y_COORD: %.3f \n", chassis.odom_y_get());
      console.printf("HEADING: %.3f \n\n", chassis.odom_theta_get());
      console.printf("BATTERY: %.0f % \n\n", pros::battery::get_capacity());
      // Drive temps
      console.println("MOTOR_TEMPS:"); //-----------------------------------> Prints the motor temperatures for each motor
      console.printf("[L1 %.0fC  ", FrontL.get_temperature()); 
      console.printf("R1 %.0fC] ", FrontR.get_temperature());
                            console.printf("\t\tAUTON [%s] \n", autoStr);
      console.printf("[L2 %.0fC  ", MidL.get_temperature());
      console.printf("R2 %.0fC] ", MidR.get_temperature());
                          console.printf("\t\tRING COLOR [%s]\n\n", ringStr);
      console.printf("[L3 %.0fC  ", BackL.get_temperature());
      console.printf("R3 %.0fC] ", BackR.get_temperature());
      // Intake & arm
      console.printf("[INTAKE %.0fC] \n", Intake.get_temperature());
      console.printf("[ARM %.0fC] \n", Arm.get_temperature());
      console.printf("[ArmDeg %i]", ArmSensor.get_position() / 100);
      pros::delay(ez::util::DELAY_TIME);
      console.clear(); //------------------------------------------------------> Refreshes screen after delay to save resources
    }
  });

  selector.run_auton();
  console.focus();
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void opcontrol() {
  // Branding for style points
  logo.focus();
  
  // Preference for driver
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  Arm.set_brake_mode(MOTOR_BRAKE_COAST);

  // Auto selects color sort
  if(selector.get_auton()->name == "BRush+")
    team = false;
  else if(selector.get_auton()->name == "QualB")
    team = false; 
  else if(selector.get_auton()->name == "Blue4-")
    team = false; 
  else if(selector.get_auton()->name =="BSWP")
    team = false;
  else if(selector.get_auton()->name == "RRush+")
    team = true;
  else if(selector.get_auton()->name == "QualR")
    team = true;
  else if(selector.get_auton()->name == "Red4-")
    team = true;
  else if(selector.get_auton()->name == "RSWP")
    team = true;
  else if(selector.get_auton()->name == "Prog")
    team = true;
  else
    sortOverride = true;

  // Prints important stuff to the controller
  pros::Task controller_screen([&]() { 
    while (true) { 
      // Initializes the strings for the info
      std::string teamStr = ""; 
      std::string autoStr = " ";

      // Gets the auton name
      if(selector.get_auton().has_value() == false){
        autoStr = "NOTHING  ";
      }else{
        autoStr = selector.get_auton()->name;
      }

      // Prints what color it will sort
      if (sortOverride)
        teamStr = "*";
      else if(team)
        teamStr = "R";
      else if(!team)
        teamStr = "B";

      // Combines all the strings and prints it as one to the controller screen
      std::string controllerString = teamStr + " A: " + autoStr + "      ";
      master.print(0, 1, "%s", controllerString);

      pros::delay(80);
    }
  });

  // Timer for the match
  pros::Task matchTimer([&]() {
    while(pros::competition::is_competition_switch()){
      pros::delay(1000);
      MatchTimer++;
    }
  });

  // Adds a timeout for the arm so it doesn't get stuck
  pros::Task armTime([&]() {
    while (true) {
      if(isMoving){
        armTimeout++;
        pros::delay(500);
      }
      // Makes it so the SWP autons don't mess up macro
      if (ArmSensor.get_position() >= grabRingPos * 100 && MatchTimer < 3){
        Arm.move_velocity(-200);
      }
      pros::delay(ez::util::DELAY_TIME);
    }
  });

  // Color sorting for the intake
  pros::Task colorSort([&]() {
    while (true) {
      if ( (optical.get_hue() >= blueLowerHue) && (optical.get_hue() <= blueHigherHue) ) {
          ringCol = false; //----------------------------> Blue
        } else if (optical.get_hue() >= redLowerHue && optical.get_hue() <= redHigherHue) {
          ringCol = true; //-----------------------------> Red
        }
        
      if(!sortOverride){ //------------------------------> Only runs if override is off
        
        optical.set_led_pwm(100); //--------------> Lights up LED for optical sensor

        // Basically if the distance sensor senses the ring close to it, it looks at what color it is and if it needs to sort
        // If it's the opposite color, the intake flicks it out near the top
        if(distanceSensor.get_distance() <= distToSensor && team == true && ringCol == false) { // Sorts blue out
          isSorting = true; //-----------------------------> Makes it so that you can't intake while its sorting
          Social1000.focus();
          Intake.move_relative(550, 600);
          pros::delay(flickDelay);
          isSorting = false;
          logo.focus();
        } else if(distanceSensor.get_distance() <= distToSensor && team == false && ringCol == true) { // Sorts red out
          isSorting = true; //-----------------------------> Makes it so that you can't intake while its sorting
          pros::delay(timeToTop);
          Social1000.focus();
          Intake.move_velocity(100);
          pros::delay(flickDelay);
          isSorting = false;
          Intake.move_velocity(600);
          logo.focus();
        }
      } else {
        optical.set_led_pwm(0);
      }
      pros::delay(ez::util::DELAY_TIME);
    }
  });

// Driver control while loop
while (true) {

//--------------------------------------------------------------Setup-------------------------------------------------------------------

    // Standard split arcade(left stick = forward-back | right stick = turning)
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    // Left button cycles back though the autons and up button cycles forward
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      selector.prev_auton(true); 
    } if (master.get_digital_new_press(DIGITAL_UP)) {
      selector.next_auton(true); 
    }

    // Pressing both X and Up at the same time will disable color sort
    if (master.get_digital(DIGITAL_X) && master.get_digital_new_press(DIGITAL_R1)) {
      sortOverride = !sortOverride;
    }

    // // Pressing X changes the team for colorsort
    // if (master.get_digital_new_press(DIGITAL_X)) {
    //   team = !team;
    // }

    // Runs selected auton when pressing X and not connected to a field
    if (master.get_digital_new_press(DIGITAL_X)) {
      autonomous();
    }

    // Puts the lady brown down at the end of the match if hanging
    if(MatchTimer > 58){
      Arm.move_velocity(-200);
    }

//--------------------------------------------------------------Pistons-----------------------------------------------------------------
    
    // Potential auto clamp
    // if(clampSensor.get_distance() <= 25){
    //   clamp_piston.set_value(true);
    // } else if(master.get_digital_new_press(DIGITAL_B)) {
    //   if (!clampPiston) {
    //     clamp_piston.set_value(true);
    //     clampPiston = !clampPiston;
    //   } else {
    //     clamp_piston.set_value(false);
    //     clampPiston = !clampPiston;
    //   }
    // }

    // Pressing B will acuate the mobile goal clamp (is toggle)
    if (master.get_digital_new_press(DIGITAL_B)) {
      if (!clampPiston) {
        clamp_piston.set_value(true);
        clampPiston = !clampPiston;
      } else {
        clamp_piston.set_value(false);
        clampPiston = !clampPiston;
      }
    }

    // Pressing Y will acuate THE DOINKER (is toggle)
    if (master.get_digital_new_press(DIGITAL_Y)) {
      if (!doinkPiston) {
        doinker_piston.set_value(true);
        doinkPiston = !doinkPiston;
      } else {
        doinker_piston.set_value(false);
        doinkPiston = !doinkPiston;
      }
    }

    // Pressing A will acuate THE DOINKER CLAMP (is toggle)
    if (master.get_digital_new_press(DIGITAL_A)) {
      if (!doinkClamp) {
        doinker_clamp.set_value(true);
        doinkClamp = !doinkClamp;
      } else {
        doinker_clamp.set_value(false);
        doinkClamp = !doinkClamp;
      }
    }

//-------------------------------------Lady Brown macro code (make sure it starts at the hardstop)--------------------------------------
    
    if(master.get_digital_new_press(DIGITAL_DOWN))
    { //--------------------------------------> Toggles between home and ring grabbing
      
      armTimeout = 0; //----------------------> Resets timeout so arm doesn't break
      isMoving = true;
      
      if(DunkPos == 0) { //-------------------> Goes from down pos (pos 0) to ring grabbing (pos 1)
        ArmSensor.reset_position();
        Arm.move_velocity(0);
        Arm.move_velocity(200);
        DunkPos = 1;
      } else { //-----------------------------> If it's not in the down pos (pos 0), then just go back to down pos
        Arm.move_velocity(0);
        Arm.move_velocity(-200);
        DunkPos = 5;
      }
    }

    if (master.get_digital_new_press(DIGITAL_RIGHT) 
     && master.get_digital(DIGITAL_DOWN) == false
     && isMoving == false) {
      
      armTimeout = 0; //----------------------> Resets timeout so arm doesn't break
      isMoving = true;

      if (DunkPos == 2) { //------------------> Goes from ring grabbing (pos 1) to scoring (pos 3)
        Social15.focus();
        Intake.move_velocity(-50);
        Arm.move_velocity(0);
        Arm.move_velocity(200);
        DunkPos = 3;
      } 
      else if (DunkPos == 4) { //-------------> Goes from scoring (pos 3) to ring grab(pos 7)
        Arm.move_velocity(0);
        Arm.move_velocity(-200);
        DunkPos = 7;
      }
      else if (DunkPos == 0) { //-------------------------------> If it's not in the scoring (pos 3) or ring grab pos (pos 1), then go to scoring
        Arm.move_velocity(0);
        Arm.move_velocity(200);
        DunkPos = 3;
      }
    }

    //----------------------------------------> Goes from home pos to ring grabbing
    if (DunkPos == 1 && (ArmSensor.get_position() >= 100 * grabRingPos)) {
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 2;
    } //--------------------------------------> Goes from ring grabbing to scored
    else if (DunkPos == 3 && (ArmSensor.get_position() >= 100 * scoreRingPos)) {
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
      Intake.move_velocity(-50);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 4;
    } //---------------------------------------> Goes from scored to ring grabbing
    else if (DunkPos == 7 && (ArmSensor.get_position() <= 100 * returnRingGrabPos)) {
      Arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      Arm.move_velocity(0);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 2;
    } //--------------------------------------> Goes from scored to home
    else if ((DunkPos == 5 && (ArmSensor.get_position() <= 100 * returnHomePos))) {
      logo.focus();
      Arm.set_brake_mode(MOTOR_BRAKE_COAST);
      Arm.move_velocity(0);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 0;
    }

    // All the timeout conditions so that the arm doesn't get stuck
    if/*---*/((ArmSensor.get_position() >= (grabRingPos * 100 - 800)) && armTimeout >= 4){
      Arm.move_velocity(0);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 2;
    } else if((ArmSensor.get_position() >= (grabRingPos * 100 + 250)) && armTimeout >= 4){
      Arm.move_velocity(0);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 2;
    } else if((ArmSensor.get_position() >= (scoreRingPos * 100)) && armTimeout >= 4){
      Arm.move_velocity(0);
      isMoving = false;
      armTimeout = 0;
      DunkPos = 4;
    }

//---------------------------------------------------------Intake code------------------------------------------------------------------

    // Pressing L1/L2 will outake :: R1/R2 will intake :: L1/R1 will outake flex
    if/*---*/((master.get_digital(DIGITAL_L1) == true 
    || master.get_digital(DIGITAL_L2) == true) ) {
      Intake.move_velocity(-600);
      IntakeFlex.move_velocity(-200);
    } else if((master.get_digital(DIGITAL_R1) == true 
    || master.get_digital(DIGITAL_R2) == true)
    && !isSorting
    && DunkPos != 3) {
      Intake.move_velocity(600);
      IntakeFlex.move_velocity(200);
    } else {
      Intake.move_velocity(0);
      IntakeFlex.move_velocity(0);
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
