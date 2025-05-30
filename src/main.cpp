#include "main.h"
#include <cstdio>
#include <string>
#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "robodash/api.h"
#include "robodash/views/console.hpp"
#include "subsystems.hpp"

//--------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------Initializing stuff, tasks, and fuctions-------------------------------------------------

//                                              <<Chassis constructor>>
ez::Drive chassis(
  {10, -9, -20},  // Left Chassis Ports 
  {-1, 2, 11},   // Right Chassis Ports 

  17,                                 // IMU Port
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

  Intake.set_brake_mode(MOTOR_BRAKE_COAST);

  optical.set_integration_time(10);

  intake_piston.set_value(false);//-----------------------> Sets intake piston to false so it starts down

  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);

  chassis.opcontrol_curve_default_set(0, 0);

  default_constants();

  chassis.initialize(); //---------------------------------> Initializes chassis and auton selector
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

//                                              <<Load images from sd>>
rd::Image logo("/usd/robotics/logo.bin", "Logo 1");

//Initialize console to display important data
rd::Console console;

//                                              <<Initialize auton selector>>
rd::Selector selector({
    {"QB-(5+1)", QualBlueNegative},
    {"QR-(5+1)", QualRedNegative},
    {"BSWP+", BluePositiveAWP},
    {"RSWP+", RedPositiveAWP},
    {"ElimB-",sixRingBlueElim},
    {"ElimR-", sixRingRedElim},
    {"B5+",BlueMiddlePositive},
    {"R5+", RedMiddlePositive},
    {"B3+",Positive3Blue},
    {"R3+", Positive3Red}
});

//                                              <<Arm PID positions>>
enum ArmPosition { //--------------------------> Enums for readability
  HOME = 0,
  GRAB_RING,
  SCORE_RING,
  AIMING,
  TIPPING_GOAL,
  NUM_POSITIONS
};

const float ArmHeights[NUM_POSITIONS] = {
  5,     //--------------------------------> Home
  140,   //--------------------------------> Loading Ring
  1100,  //--------------------------------> Scoring Ring
  1500,  //--------------------------------> Aiming
  1700,  //--------------------------------> Tipping Goal
};

// Sets the default position to down
static ArmPosition currentArmPosition = HOME;

// Arm PID task; Always runs in the background to move the arm to desired target
void arm_task() {
  pros::delay(2500);  // Set EZ-Template calibrate before this function starts running
  
  while (true) {
    set_arm(armPID.compute(ArmL.get_position()));

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Arm_Task(arm_task); 

// Color sorting for the intake
void color_sort() {
  pros::delay(2500);

  while (!isIntakeOverheated()) { //-------------------> Shuts off if the intake is too hot
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
pros::Task Color_Sort(color_sort); 

void auto_clamp() {
  pros::delay(2500);  // Set EZ-Template calibrate before this function starts running
  
  while (true) {
    if(letGoMogo)
      clamp_piston.set_value(false);
    else if(clampSensor.get_distance() <= distToSensor)
      clamp_piston.set_value(true);
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task autoClamp(auto_clamp); 

// Prints important stuff to the brain
void console_display(){ 
  pros::delay(2500);
  while (true) {
    // Odom stuff
    console.printf("X_COORD: [%.3f]   ",chassis.odom_x_get()); //---------> x, y, and heading values are printed
    console.printf("Y_COORD: [%.3f]\n", chassis.odom_y_get());
    console.printf("HEADING: [%.3f] \n\n", chassis.odom_theta_get());
    console.printf("BATTERY: %.0f % \n\n", pros::battery::get_capacity());
    // Drive temps
    console.println("MOTOR_TEMPS:"); //-----------------------------------> Prints the motor temperatures for each motor
    console.printf("[L1 %.0fC  ", FrontL.get_temperature()); 
    console.printf("R1 %.0fC] \n", FrontR.get_temperature());
    console.printf("[L2 %.0fC  ", MidL.get_temperature());
    console.printf("R2 %.0fC] \n", MidR.get_temperature());
    console.printf("[L3 %.0fC  ", BackL.get_temperature());
    console.printf("R3 %.0fC] \n\n", BackR.get_temperature());
    // Intake & arm
    console.printf("[ARM %.0fC]\n", (ArmL.get_temperature()));
    console.printf("[INTAKE: %.0fC]", checkIntakeTemp());
    
    pros::delay(ez::util::DELAY_TIME);
    console.clear(); //--------------------------------------------------------> Refreshes screen after delay to save resources
  }
}
pros::Task ConsoleUpdate(console_display);

// Prints important stuff to the controller screen
void controller_display() {
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
  else if(team == RED_TEAM)
    teamStr = "R";
  else
    teamStr = "B";

  // Combines all the strings and prints it as one to the controller screen
  std::string controllerString = teamStr + " A: " + autoStr + "      ";
  master.print(0, 1, "%s", (controllerString));
}

// Arm functions to move between the driver control arm presets
void next_arm_preset() {
  currentArmPosition = static_cast<ArmPosition>((currentArmPosition + 1) % NUM_POSITIONS);
  armPID.target_set(ArmHeights[currentArmPosition]);
}
void prev_arm_preset() {
  // Add NUM_POSITIONS before modulo to handle negative case
  currentArmPosition = static_cast<ArmPosition>((currentArmPosition - 1 + NUM_POSITIONS) % NUM_POSITIONS);
  armPID.target_set(ArmHeights[currentArmPosition]);
}

// Auto selects color sort
void auto_color_sort_select() {
  if/*-*/(selector.get_auton()->name == "BRush+")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "QB-(5+1)")
    team = BLUE_TEAM; 
  else if(selector.get_auton()->name == "ElimB-")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "B5+")
    team = BLUE_TEAM;  
  else if(selector.get_auton()->name == "B3+")
    team = BLUE_TEAM; 
  else if(selector.get_auton()->name =="BSWP+")
    team = BLUE_TEAM;
  else if(selector.get_auton()->name == "RRush+")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "QR-(5+1)")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "ElimR-")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "R5+")
    team = RED_TEAM;
  else if(selector.get_auton()->name == "R3+")
    team = RED_TEAM; 
  else if(selector.get_auton()->name == "RSWP+")
    team = RED_TEAM;
  else
    sortOverride = true;
}

// Sets the starting position of the arm for autons
void set_starting_arm_position() {
  if(selector.get_auton().has_value() == false 
  || selector.get_auton()->name == "B5+" 
  || selector.get_auton()->name == "R5+"
  || selector.get_auton()->name == "B3+" 
  || selector.get_auton()->name == "R3+") {
    currentArmPosition = HOME;
    armPID.target_set(ArmHeights[currentArmPosition]);
  } else {
    currentArmPosition = GRAB_RING;
    armPID.target_set(ArmHeights[currentArmPosition]);
  }
}

// All of the button inputs for the controller in driver control
void controls() {
//-----------------------------------------------------------------Setup----------------------------------------------------------------
  // Left button cycles autons
  if (master.get_digital_new_press(DIGITAL_LEFT)) {
    selector.next_auton(true); 
  } 

  // If not connected to a comp switch, up button runs auto, otherwise it cycles back autons
  if (pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_UP)) {
      selector.prev_auton(true);
    }
  } else if(!pros::competition::is_connected()){
    if (master.get_digital_new_press(DIGITAL_UP)) {
      autonomous();
    }
  }

//--------------------------------------------------------------Pistons-----------------------------------------------------------------
  
  // Pressing Y will acuate left doinker (is toggle)
  if(master.get_digital_new_press(DIGITAL_Y)) {
    if (!doinkPistonL) {
      doinker_left.set_value(true);
      doinkPistonL = !doinkPistonL;
    } else {
      doinker_left.set_value(false);
      doinkPistonL = !doinkPistonL;
    }
  }
  
  // Pressing B will acuate right doinker (is toggle)
  if (master.get_digital_new_press(DIGITAL_B)) {
    if (!doinkPistonR) {
      doinker_right.set_value(true);
      doinkPistonR = !doinkPistonR;
    } else {
      doinker_right.set_value(false);
      doinkPistonR = !doinkPistonR;
    }
  }

  // Pressing A will acuate right doinker (is toggle)
  if (master.get_digital_new_press(DIGITAL_A)) {
    if (!intakePiston) {
      intake_piston.set_value(true);
      intakePiston = !intakePiston;
    } else {
      intake_piston.set_value(false);
      intakePiston = !intakePiston;
    }
  }

  // Auto clamp will deactivate if L2 is pressed
  if (master.get_digital(DIGITAL_L2)) {
    disableAutoClamp();
  } else {
    enableAutoClamp();
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
    if (master.get_digital(DIGITAL_R1)) {
      Intake.move(127);
    } else if (master.get_digital(DIGITAL_R2)) {
      Intake.move(-127);
    } else {
      Intake.move(0);
    }
  } 

  if (master.get_digital_new_press(DIGITAL_L1)) {
    sortOverride = !sortOverride;
  }
   
  if (master.get_digital_new_press(DIGITAL_B)) {
    toggleColorSort();
  }
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

//--------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------Auton and Driver Control-------------------------------------------------------------

void opcontrol() {
  // Branding for style points
  console.focus();

  // Preference for driver
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.pid_tuner_disable(); 

  // Automatically sets starting arm position for auto
  set_starting_arm_position();

  // Automatically sets color sorting based on auton
  auto_color_sort_select();

  distToSensor = 10;

  // Driver control while loop
  while (true) {
    
    // Standard split arcade(left stick = forward-back | right stick = turning)
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    // Displays important information on the controller (Color sort status and Auton selection)
    controller_display();

    // Takes care of all the button tasks 
    controls();

    // This is used for timer calculations keep it ez::util::DELAY_TIME
    pros::delay(ez::util::DELAY_TIME);   
  }
}

void autonomous() {
  // Anti jam for the intake; If the motor torque is high and it isn't moving, it moves back briefly
  // pros::Task anti_jam{[=]{
  //   while (!isIntakeOverheated()) {
  //     if(Intake.get_efficiency() <= 25 && Intake.get_torque() >= 1.16){
  //       float pastVoltage = Intake.get_voltage();
  //       isSorting = true;
  //       Intake.move(-127);
  //       pros::delay(100);
  //       Intake.move(pastVoltage);
  //       isSorting = false;
  //     }
  //     pros::delay(ez::util::DELAY_TIME);
  //   }
  // }};
  distToSensor = 6;
  chassis.pid_targets_reset();                          // Resets PID targets to 0
  chassis.drive_imu_reset();                            // Reset gyro position to 0
  chassis.drive_sensor_reset();                         // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);// Set motors to hold. This helps autonomous consistency
  Intake.set_brake_mode(MOTOR_BRAKE_COAST);
  console.focus();
  selector.run_auton();
}