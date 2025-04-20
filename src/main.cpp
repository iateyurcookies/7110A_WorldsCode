#include "main.h"
#include <string>
#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robodash/api.h"
#include "robodash/views/console.hpp"
#include "subsystems.hpp"

//---------------------------------------------------Initialize important variables-----------------------------------------------------

//                                                    <<Arm PID positions>>
enum ArmPosition { //--------------------------------> Enums for readability
  HOME = 0,
  GRAB_RING,
  SCORE_RING,
  AIMING,
  TIPPING_GOAL,
  NUM_POSITIONS
};

const float ArmHeights[NUM_POSITIONS] = {
  5,    //---------------------------------------> Home
  150,   //--------------------------------------> Loading Ring
  1080,  //--------------------------------------> Scoring Ring
  1500,  //--------------------------------------> Aiming
  1700,  //--------------------------------------> Tipping Goal
};

static ArmPosition currentArmPosition = HOME; //----> Sets the default position to down

//--------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------Initialize other stuff---------------------------------------------------------------

//                                         <<Load images from sd>>
// rd::Image logo("/usd/robotics/logo2.bin", "Logo 1");
// rd::Image logo2("/usd/robotics/logo.bin", "Logo 2");
// rd::Image Social15("/usd/robotics/+15.bin", "+15");
// rd::Image Social1000("/usd/robotics/-100.bin", "+100");
// rd::Image dhyan("/usd/robotics/dhyan.bin", "Dhyan");

//Initialize console to display important data
rd::Console console;

//                     <<Initialize auton selector>>
rd::Selector selector({
    {"Testing", test},
    {"BRush+", BlueLeftRush},
    {"RRush+", RedRightRush},
    {"QualB", QualBlueLeftRush},
    {"QualR", QualRedRightRush},
    {"Blue4-",sixRingBlue},
    {"Red4-", sixRingRed},
    {"BMid-",BlueMiddleNegative},
    {"RMid-", RedMiddleNegative},
    {"BMid+",BlueMiddlePositive},
   {"RMid+", RedMiddlePositive},
   {"BSWP-", BlueRightAWP},
   {"RSWP-", RedLeftAWP}
});

//                                              <<Chassis constructor>>
ez::Drive chassis(
  {10, -9, -20},   // Left Chassis Ports 
  {-1, 2, 11},    // Right Chassis Ports 

  14,                                  // IMU Port
  3.25,                         // Wheel Diameter
  450                                    // Wheel RPM
);

//                                              <<Odom tracking setup>>
// you should get positive values on the encoders going FORWARD and RIGHT
// This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel horiz_tracker(8, 2, 1.85); 

// This tracking wheel is parallel to the drive wheels
ez::tracking_wheel vert_tracker(-7, 2, .06); 

void initialize() {
  pros::delay(500); //-----------------------> Stop the user from doing anything while legacy ports configure
  armPID.exit_condition_set(     //------------------------> Sets exit conditions for the pid
    90, 
    3, 
    250, 
    7, 
    500, 
    500);
  ArmL.tare_position();          //-----------------------> Initializes the motor encoder for the arm pid
  ArmL.set_brake_mode(MOTOR_BRAKE_HOLD);
  ArmR.set_brake_mode(MOTOR_BRAKE_HOLD);
  intakeRotation.reset();

  doinker_clamp.set_value(true); //-----------------------> Sets the doinker clamp piston to false so it starts closed
  intake_piston.set_value(false);//-----------------------> Sets intake piston to false so it starts down

  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);

  chassis.opcontrol_curve_default_set(0, 0);

  default_constants();

  chassis.initialize(); //---------------------------------> Initializes chassis and auton selector
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  pros::screen::erase();
}

void arm_task() {
  pros::delay(1500);  // Set EZ-Template calibrate before this function starts running
  
  while (true) {
    set_arm(armPID.compute(ArmL.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Arm_Task(arm_task); 

// Color sorting for the intake
void color_sort() {
  pros::delay(1500);

  while (true) {
    // Basically if the distance sensor senses the ring close to it, it looks at what color it is and if it needs to sort
    currentRingColor = getCurrentRingColor();

    if(!sortOverride){ //------------------------------> Only runs if override is off
      optical.set_led_pwm(100); //--------------> Lights up LED for optical sensor
      if(currentRingColor != NONE)
        sortRing(currentRingColor);
    } else { //----------------------------------------> Turns off led if color sort is off
      optical.set_led_pwm(0);
    }
    
    pros::delay(ez::util::DELAY_TIME);

    }
  }
// pros::Task Color_Sort(color_sort); 

void console_display(){   //-------------------------> printing important data to the brain
  pros::delay(2000);
  while (true) {
    // Odom stuff
    console.printf("X_COORD: %.3f \n",chassis.odom_x_get()); //------------> x, y, and heading values are printed
    console.printf("Y_COORD: %.3f \n", chassis.odom_y_get());
    console.printf("HEADING: %.3f \n\n", chassis.odom_theta_get());
    console.printf("BATTERY: %.0f % \n\n", pros::battery::get_capacity());
    // Drive temps
    console.println("MOTOR_TEMPS:"); //-----------------------------------> Prints the motor temperatures for each motor
    console.printf("[L1 %.0fC  ", FrontL.get_temperature()); 
    console.printf("R1 %.0fC] ", FrontR.get_temperature());
    console.printf("[L2 %.0fC  ", MidL.get_temperature());
    console.printf("R2 %.0fC] ", MidR.get_temperature());
    console.printf("[L3 %.0fC  ", BackL.get_temperature());
    console.printf("R3 %.0fC] ", BackR.get_temperature());
    // Intake & arm
    console.printf("[INTAKE %.0fC] \n", Intake.get_temperature());
    console.printf("[ARM %.0fC] \n", ArmL.get_temperature());
    

    pros::delay(ez::util::DELAY_TIME);
    console.clear(); //------------------------------------------------------> Refreshes screen after delay to save resources
  }
}
pros::Task ConsoleUpdate(console_display);

void auto_color_sort_select() {
  // Auto selects color sort
  if/*-*/(selector.get_auton()->name == "BRush+")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "BRushTug")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "QualB")
    team = BLUE_TEAM; 
  else if(selector.get_auton()->name == "Blue4-")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "Blue4-M")
    team = BLUE_TEAM;  
  else if(selector.get_auton()->name =="BSWP")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "RRush+")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "RRushTug")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "QualR")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "Red4-")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "Red4-M")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "RSWP")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "Prog")
    team = RED_TEAM;
  else
    sortOverride = true;
}

// void antiJam() {
//   if(Intake.get_efficiency() == 0) {
//     isSorting = true;
//     Intake.move_velocity(-400);
//     pros::delay(20);
//     Intake.move_velocity(400);
//     isSorting = false;
//   }
// }
// pros::Task AntiJamTask(antiJam);

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

//--------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------Auton and Driver Control-------------------------------------------------------------

void controller_display() {
  // Initializes the strings for the info
  std::string teamStr = ""; 
  std::string ringStr = "";
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
  else if(team == RED_TEAM)
    teamStr = "R";
  else
    teamStr = "B";

  if(currentRingColor == RED)
    ringStr = "RED";
  else
    ringStr = "BLUE";

  // Combines all the strings and prints it as one to the controller screen
  std::string controllerString = teamStr + " A: " + autoStr + "      ";
  master.print(0, 1, "%s", (controllerString));
}

void next_arm_preset() {
  currentArmPosition = static_cast<ArmPosition>((currentArmPosition + 1) % NUM_POSITIONS);
  armPID.target_set(ArmHeights[currentArmPosition]);
}

void prev_arm_preset() {
  // Add NUM_POSITIONS before modulo to handle negative case
  currentArmPosition = static_cast<ArmPosition>((currentArmPosition - 1 + NUM_POSITIONS) % NUM_POSITIONS);
  armPID.target_set(ArmHeights[currentArmPosition]);
}

void controls() {
//-----------------------------------------------------------------Setup----------------------------------------------------------------
  // Left button cycles autons
  if (master.get_digital_new_press(DIGITAL_LEFT)) {
    selector.next_auton(true); 
  } 

  if (pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_UP)) {
      selector.prev_auton(true);
    }
  } else if(!pros::competition::is_connected()){
    if (master.get_digital_new_press(DIGITAL_UP)) {
      autonomous();
    }
  }

  if (master.get_digital_new_press(DIGITAL_L1))
    toggleColorSort();

  if (master.get_digital_new_press(DIGITAL_L2)){
    sortOverride = !sortOverride;
  }

//--------------------------------------------------------------Pistons-----------------------------------------------------------------
  
  // Potential auto clamp
  if(master.get_digital_new_press(DIGITAL_Y)) {
    clamp_piston.set_value(false);
    clampPiston = false;
  } else if (clampSensor.get_distance() <= distToSensor && !master.get_digital(DIGITAL_Y)) {
    clamp_piston.set_value(true);
    clampPiston = true;
  }
  
  // // Pressing A will acuate the mobile goal clamp (is toggle)
  // if (master.get_digital_new_press(DIGITAL_Y)) {
  //   if (!clampPiston) {
  //     clamp_piston.set_value(true);
  //     clampPiston = !clampPiston;
  //   } else {
  //     clamp_piston.set_value(false);
  //     clampPiston = !clampPiston;
  //   }
  // }
  
  // Pressing Y will acuate THE DOINKER (is toggle)
  if (master.get_digital_new_press(DIGITAL_B)) {
    if (!doinkPiston) {
      doinker_piston.set_value(true);
      doinkPiston = !doinkPiston;
    } else {
      doinker_piston.set_value(false);
      doinkPiston = !doinkPiston;
    }
  }
  
  // Pressing B will acuate THE DOINKER CLAMP (is toggle)
  if (master.get_digital_new_press(DIGITAL_A)) {
    if (!doinkClamp) {
      doinker_clamp.set_value(true);
      doinkClamp = !doinkClamp;
    } else {
      doinker_clamp.set_value(false);
      doinkClamp = !doinkClamp;
    }
  }

//------------------------------------------------------------Arm code------------------------------------------------------------------

  if (master.get_digital_new_press(DIGITAL_RIGHT)) {
    next_arm_preset();
  }

  if (master.get_digital_new_press(DIGITAL_DOWN)) {
    prev_arm_preset();
  }

//---------------------------------------------------------Intake code------------------------------------------------------------------

  // Pressing R2 will intake, pressing R1 will outake, code stops if its color sorting
  if(!isSorting) {
    if(master.get_digital(DIGITAL_R1)) {
      Intake.move(127);
    } else if(master.get_digital(DIGITAL_R2)) {
      Intake.move(-127);
    } else {
    Intake.move(0);
  }
} 
}

void opcontrol() {
  // Branding for style points
  // logo.focus();

  // Preference for driver
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // Automatically sets color sorting based on auton
  auto_color_sort_select();

  chassis.pid_tuner_disable();

  // Driver control while loop
  while (true) {

    // Standard split arcade(left stick = forward-back | right stick = turning)
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    // Displays important information on the controller (Color sort status and Auton selection)
    controller_display();

    // Takes care of all the button tasks 
    controls();

    pros::delay(ez::util::DELAY_TIME);   // This is used for timer calculations keep it ez::util::DELAY_TIME
  }
}

void autonomous() {
  chassis.pid_targets_reset();                          // Resets PID targets to 0
  chassis.drive_imu_reset();                            // Reset gyro position to 0
  chassis.drive_sensor_reset();                         // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);// Set motors to hold. This helps autonomous consistency
  console.focus();
  selector.run_auton();
}