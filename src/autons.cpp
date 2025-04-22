#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"


// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

// Arm positions
const float HOME = 5;
const float LOADING = 150;
const float SCORING = 1080;
const float TIPPING = 1700;
const float SCORE_ALLIANCE = 1400;

// Use this to intake in auto with color sort
void intakeWithSort() {
  if(!isSorting)
    Intake.move(127);
}

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.5, 0.05, 30.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(8, 0.0, 60);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(
    90_ms, 
    3_deg,
    250_ms, 
    7_deg, 
    500_ms, 
    500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(
    90_ms, 
    1_in, 
    250_ms, 
    3_in, 
    500_ms, 
    500_ms);
  chassis.pid_odom_turn_exit_condition_set(
    90_ms, 
    3_deg, 
    250_ms, 
    7_deg,
    500_ms, 
    750_ms);
  chassis.pid_odom_drive_exit_condition_set(
    90_ms, 
    1_in, 
    250_ms,
    3_in, 
    500_ms, 
    750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  //IMU constant
  chassis.drive_imu_scaler_set(1.006);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(.6);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.5);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void sixRingBlueElim(){
  chassis.pid_odom_behavior_set(ez::shortest);
  chassis.odom_xyt_set(0_in, 0_in, 54.6_deg); 
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-32_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn to 4 stack and start intake
  chassis.pid_turn_set(-130, 90);
  chassis.pid_wait();
  intakeWithSort();

  //Swing into 4 stack and intake 2 rings
  chassis.pid_odom_set(8_in, 90);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, -95, 100, 20);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(4_in, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(16_in, 50);
  chassis.pid_wait();
  pros::delay(250);
  
  //Back up and turn towards 2 stack
  chassis.pid_odom_set(-8_in, 90);
  chassis.pid_wait();
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait_quick_chain();
  
  //Intake 2 stack ring
  chassis.pid_odom_set(44_in, 75);
  chassis.pid_wait_quick();
  
  //Turn towards corner and get bottom ring + 2nd corner ring
  chassis.pid_turn_set(-52, 90);
  chassis.pid_wait_quick();
  intakeWithSort();
  chassis.pid_odom_set(14_in, 75);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_odom_set(-22_in, 50);
  chassis.pid_wait();

  //Back up and turn towards alliance 2 stack
  chassis.pid_turn_set(90, 90);
  chassis.pid_wait();

  //Go to 2 stack and intake top ring
  intakeWithSort();
  chassis.pid_odom_set(36_in, 127);
  chassis.pid_wait();
  
  //Drive full speed to positive corner

}

void sixRingRedElim(){
  chassis.pid_odom_behavior_set(ez::shortest);
  chassis.odom_xyt_set(0_in, 0_in, -54.6_deg); 
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-34_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn to 4 stack and start intake
  chassis.pid_turn_set(130, 90);
  chassis.pid_wait();
  intakeWithSort();

  //Swing into 4 stack and intake 2 rings
  chassis.pid_odom_set(10_in, 90);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 95, 100, 20);
  chassis.pid_wait_quick();
  chassis.pid_odom_set(16_in, 127);
  chassis.pid_wait();
  pros::delay(250);
  
  //Back up and turn towards 2 stack
  chassis.pid_odom_set(-6_in, 90);
  chassis.pid_wait();
  Intake.move(0);
  chassis.pid_turn_set(0, 80);
  chassis.pid_wait_quick_chain();
  
  //Intake 2 stack ring
  chassis.pid_odom_set(46_in, 35);
  intakeWithSort();
  chassis.pid_wait_until(36_in);
  chassis.pid_speed_max_set(90);
  chassis.pid_wait();
  
  //Turn towards corner and get bottom ring
  chassis.pid_turn_set(52, 90);
  chassis.pid_wait();
  intakeWithSort();
  chassis.pid_odom_set(12_in, 45);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_odom_set(-18_in, 60);
  chassis.pid_wait();

  //Back up and turn towards alliance 2 stack
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait();

  //Go to 2 stack and intake top ring
  intakeWithSort();
  chassis.pid_odom_set(40_in, 127);
  chassis.pid_wait();

  //Drive full speed to positive corner

}

void BlueLeftRush(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Raise the arm up and drive forward while running intake

  //Once arm has ring seated raise it up enough to keep intaking 

  //Drive straight to rush goal and score top ring w lady brown

  //Turn while in scoring position to tip

  //Return arm to home position and turn towards mogo

  //Go to mogo and clamp

  //Score the two stack ring that you have in your intake

  //Drive to alliance stake 2 stack and get the top ring from that

  //Turn around and go to positive corner

  //Get both corner rings and score them

  //Sweep corner

  //Turn and sit in corner

}

void RedRightRush(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Raise the arm up and drive forward while running intake

  //Once arm has ring seated raise it up enough to keep intaking 

  //Drive straight to rush goal and score top ring w lady brown

  //Turn while in scoring position to tip

  //Return arm to home position and turn towards mogo

  //Go to mogo and clamp

  //Score the two stack ring that you have in your intake

  //Drive to alliance stake 2 stack and get the top ring from that

  //Turn around and go to positive corner

  //Get both corner rings and score them

  //Sweep corner

  //Turn and sit in corner

}

void BlueMiddlePositive(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, 54.6_deg); 

  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);
  
  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-32_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn towards center rings

  //Drive to center rings while outtaking and push the two red rings on our side away

  //Turn towards blue rings and put down doinker

  //Move back with blue center rings

  //Put up doinker and intake 2 rings

  //Turn around to 2 stack

  //Go to 2 stack and intake ring

  //Turn to corner

  //Intake and drive to corner to get 2 corner rings

  //Sweep corner and sit

}

void RedMiddlePositive(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, -54.6_deg); 

  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);
  
  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-32_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn towards center rings

  //Drive to center rings while outtaking and push the two blue rings on our side away

  //Turn towards red rings and put down doinker

  //Move back with red center rings

  //Put up doinker and intake 2 rings

  //Turn around to 2 stack

  //Go to 2 stack and intake ring

  //Turn to corner

  //Intake and drive to corner to get 2 corner rings

  //Sweep corner and sit
  
}

void QualBlueNegative(){
  chassis.pid_odom_behavior_set(ez::shortest);
  chassis.odom_xyt_set(0_in, 0_in, 54.6_deg); 
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-32_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn to 4 stack and start intake
  chassis.pid_turn_set(-130, 90);
  chassis.pid_wait();
  intakeWithSort();

  //Swing into 4 stack and intake 2 rings
  chassis.pid_odom_set(8_in, 90);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, -95, 100, 20);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(4_in, 100);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(16_in, 50);
  chassis.pid_wait();
  pros::delay(250);
  
  //Back up and turn towards 2 stack
  chassis.pid_odom_set(-8_in, 90);
  chassis.pid_wait();
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait_quick_chain();
  
  //Intake 2 stack ring
  chassis.pid_odom_set(44_in, 75);
  chassis.pid_wait_quick();
  
  //Turn towards corner and get bottom ring
  chassis.pid_turn_set(-52, 90);
  chassis.pid_wait_quick();
  intakeWithSort();
  chassis.pid_odom_set(14_in, 75);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_odom_set(-22_in, 50);
  chassis.pid_wait();

  //Back up and turn towards alliance 2 stack
  chassis.pid_turn_set(90, 90);
  chassis.pid_wait();

  //Go to 2 stack and intake top ring
  intakeWithSort();
  chassis.pid_odom_set(36_in, 127);
  chassis.pid_wait();

  //Touch ladder
  armPID.target_set(SCORE_ALLIANCE + 100);
  chassis.pid_turn_set(-180, 90);
  chassis.pid_wait();
  chassis.pid_odom_set(14_in, 40);
  chassis.pid_wait();
}

void QualRedNegative(){
  chassis.pid_odom_behavior_set(ez::shortest);
  chassis.odom_xyt_set(0_in, 0_in, -54.6_deg); 
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-34_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn to 4 stack and start intake
  chassis.pid_turn_set(130, 90);
  chassis.pid_wait();
  intakeWithSort();

  //Swing into 4 stack and intake 2 rings
  chassis.pid_odom_set(10_in, 90);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 95, 100, 20);
  chassis.pid_wait_quick();
  chassis.pid_odom_set(16_in, 127);
  chassis.pid_wait();
  pros::delay(250);
  
  //Back up and turn towards 2 stack
  chassis.pid_odom_set(-6_in, 90);
  chassis.pid_wait();
  Intake.move(0);
  chassis.pid_turn_set(0, 80);
  chassis.pid_wait_quick_chain();
  
  //Intake 2 stack ring
  chassis.pid_odom_set(46_in, 35);
  intakeWithSort();
  chassis.pid_wait_until(36_in);
  chassis.pid_speed_max_set(90);
  chassis.pid_wait();
  
  //Turn towards corner and get bottom ring
  chassis.pid_turn_set(52, 90);
  chassis.pid_wait();
  intakeWithSort();
  chassis.pid_odom_set(12_in, 45);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_odom_set(-18_in, 60);
  chassis.pid_wait();

  //Back up and turn towards alliance 2 stack
  chassis.pid_turn_set(-90, 75);
  chassis.pid_wait();

  //Go to 2 stack and intake top ring
  intakeWithSort();
  chassis.pid_odom_set(40_in, 127);
  chassis.pid_wait();

  //Touch ladder
  armPID.target_set(SCORE_ALLIANCE + 100);
  chassis.pid_turn_set(180, 90);
  chassis.pid_wait();
  chassis.pid_odom_set(14_in, 40);
  chassis.pid_wait();
}

void BluePositiveAWP(){
  chassis.pid_odom_behavior_set(ez::shortest);
  
  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, 54.6_deg); 
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-34_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn to alliance 2 stack

  //Get top ring from 2 stack

  //Go backwards and turn towards center rings

  //Go to closest center ring and put down doinker

  //Go back with doinker down, put up doinker, turn, and intake center ring

  //Turn towards 2 stack

  //Go to 2 stack and intake ring

  //Stop intake about halfway up and drop mogo while turning to rush mogo

  //Go backwards to rush mogo and clamp

  //Score 2 stack ring

  //Touch ladder
  
}

void RedPositiveAWP(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, -54.6_deg); 
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  //Move back and turn towards mogo
  chassis.pid_odom_set(-14_in, 75);
  chassis.pid_wait();
  armPID.target_set(HOME);
  
  chassis.pid_turn_set(0, 90);
  chassis.pid_wait();
  
  chassis.pid_targets_reset();                        
  chassis.drive_imu_reset();                           
  chassis.drive_sensor_reset();  
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); 
  pros::delay(250);

  //Drive to mogo and clamp
  chassis.pid_odom_set(-34_in, 120);
  chassis.pid_wait_until(-12_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_until(-32_in);
  clamp_piston.set_value(true);
  chassis.pid_wait();

  //Turn to alliance 2 stack

  //Get top ring from 2 stack

  //Go backwards and turn towards center rings

  //Go to closest center ring and put down doinker

  //Go back with doinker down, put up doinker, turn, and intake center ring

  //Turn towards 2 stack

  //Go to 2 stack and intake ring

  //Stop intake about halfway up and drop mogo while turning to rush mogo

  //Go backwards to rush mogo and clamp

  //Score 2 stack ring

  //Touch ladder

}

void test(){
  chassis.pid_odom_behavior_set(ez::shortest);

  chassis.pid_turn_set(90_deg, 90);
  chassis.pid_wait();

}

void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

// Interference example
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// Odom Drive PID
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

// Odom Pure Pursuit
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

// Odom Pure Pursuit Wait Until
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

// Odom Boomerang
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

// Odom Boomerang Injected Pure Pursuit
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

// // Calculate the offsets of your tracking wheels
// void measure_offsets() {
//   // Number of times to test
//   int iterations = 10;

//   // Our final offsets
//   double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

//   // Reset all trackers if they exist
//   if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
//   if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
//   if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
//   if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
//   for (int i = 0; i < iterations; i++) {
//     // Reset pid targets and get ready for running an auton
//     chassis.pid_targets_reset();
//     chassis.drive_imu_reset();
//     chassis.drive_sensor_reset();
//     chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

//     double imu_start = chassis.odom_theta_get();
//     double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

//     // Turn to target at half power
//     chassis.pid_turn_set(target, 63, ez::raw);
//     chassis.pid_wait();
//     pros::delay(250);

//     // Calculate delta in angle
//     double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

//     // Calculate delta in sensor values that exist
//     double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
//     double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
//     double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
//     double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

//     // Calculate the radius that the robot traveled
//     l_offset += l_delta / t_delta;
//     r_offset += r_delta / t_delta;
//     b_offset += b_delta / t_delta;
//     f_offset += f_delta / t_delta;
//   }

//   // Average all offsets
//   l_offset /= iterations;
//   r_offset /= iterations;
//   b_offset /= iterations;
//   f_offset /= iterations;

//   // Set new offsets to trackers that exist
//   if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
//   if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
//   if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
//   if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
// }
