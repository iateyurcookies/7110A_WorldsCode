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
const float LOADING = 140;
const float SCORING = 1080;
const float TIPPING = 1700;
const float SCORE_ALLIANCE = 1400;

// Use this to intake in auto with color sort
void intakeWithSort() {
  if(!isSorting)
    Intake.move_velocity(200);
}

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);               // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);              // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.5, 0.05, 35.0, 15.0);// Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);                 // Swing constants
  chassis.pid_odom_angular_constants_set(9, 0.0, 62);              // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5, 0.0, 40);            // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(
    90_ms, 
    3_deg,
    250_ms, 
    7_deg, 
    275_ms, 
    500_ms);
  chassis.pid_swing_exit_condition_set(
    90_ms,
    3_deg, 
    250_ms, 
    7_deg, 
    275_ms, 
    500_ms);
  chassis.pid_drive_exit_condition_set(
    90_ms, 
    1_in, 
    250_ms, 
    3_in, 
    275_ms, 
    500_ms);
  chassis.pid_odom_turn_exit_condition_set(
    90_ms, 
    3_deg, 
    250_ms, 
    7_deg,
    275_ms, 
    750_ms);
  chassis.pid_odom_drive_exit_condition_set(
    90_ms, 
    1_in, 
    250_ms,
    3_in, 
    275_ms, 
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
  chassis.odom_turn_bias_set(.8);

  chassis.odom_look_ahead_set(7_in);         // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);// This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.6);          // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);// Changes the default behavior for turning, this defaults it to the shortest path there
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

void BlueMiddlePositive(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, -30_deg); 
  enableAutoClamp();

  // Drive to mogo and clamp
  chassis.pid_drive_set(-38_in, 90, true);
  chassis.pid_wait_until(-18_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait_quick();
  armPID.target_set(HOME);

  //Turn towards center rings
  intakeWithSort();
  chassis.pid_turn_set(-129, 85, true);
  chassis.pid_wait_quick();

  //Drive to center rings
  chassis.pid_drive_set(13_in, 90, true);
  chassis.pid_wait_until(10_in);
  Intake.move(0);
  chassis.pid_wait();
  
  //Turn towards red rings and put down doinker
  doinker_left.set_value(true);

  //Move back with red center rings
  chassis.pid_drive_set(-21_in, 110, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, -184, 100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-16_in, 120, true);
  chassis.pid_wait_quick();

  //Put up doinker and intake 2 rings
  doinker_left.set_value(false);
  pros::delay(200);
  intakeWithSort();

  //Turn around to 2 stack
  chassis.pid_turn_set(-114, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, -275, 100, 5);
  chassis.pid_wait_quick_chain();

  //Go to 2 stack and intake ring
  chassis.pid_drive_set(20_in, 75, true);
  chassis.pid_wait_quick();
  pros::delay(500);
  
  //Turn to and go to corner
  chassis.pid_swing_set(ez::RIGHT_SWING, -360, 100, 10);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(28_in, 127, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-320, 90, true);
  chassis.pid_wait_quick();

  //Intake and drive to corner to get 2 corner rings
  chassis.pid_drive_set(8_in, 90, true);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_drive_set(-6_in, 80, true);
  chassis.pid_wait();
  intake_piston.set_value(true);
  chassis.pid_drive_set(6_in, 80, true);
  chassis.pid_wait();
  pros::delay(550);
  intake_piston.set_value(false);
  
  //Sweep corner
  chassis.pid_drive_set(-7_in, 90, true);
  chassis.pid_wait_quick();
  doinker_left.set_value(true);
  chassis.pid_turn_behavior_set(longest);
  chassis.pid_turn_set(-140, 90, true);
  chassis.pid_wait_quick();
  disableAutoClamp();
  doinker_left.set_value(false);

  //Drop mogo in corner and set up for steal
  chassis.pid_drive_set(-10_in, 90, true);
  chassis.pid_wait_quick();
  Intake.move(0);  
}

void RedMiddlePositive(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, 30_deg); 
  enableAutoClamp();

  // Drive to mogo and clamp
  chassis.pid_drive_set(-38_in, 90, true);
  chassis.pid_wait_until(-18_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait_quick();
  armPID.target_set(HOME);

  //Turn towards center rings
  intakeWithSort();
  chassis.pid_turn_set(129, 85, true);
  chassis.pid_wait_quick();

  //Drive to center rings
  chassis.pid_drive_set(15.5_in, 90, true);
  chassis.pid_wait_until(9_in);
  Intake.move(0);
  chassis.pid_wait();
  
  //Turn towards red rings and put down doinker
  doinker_right.set_value(true);

  //Move back with red center rings
  chassis.pid_drive_set(-21_in, 110, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 184, 100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-16_in, 120, true);
  chassis.pid_wait_quick();

  //Put up doinker and intake 2 rings
  doinker_right.set_value(false);
  pros::delay(200);
  intakeWithSort();

  //Turn around to 2 stack
  chassis.pid_turn_set(114, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 275, 100, 5);
  chassis.pid_wait_quick_chain();

  //Go to 2 stack and intake ring
  chassis.pid_drive_set(20_in, 75, true);
  chassis.pid_wait_quick();
  pros::delay(500);
  
  //Turn to and go to corner
  chassis.pid_swing_set(ez::LEFT_SWING, 362, 100, 10);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(28_in, 127, true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(320, 90, true);
  chassis.pid_wait_quick();

  //Intake and drive to corner to get 2 corner rings
  chassis.pid_drive_set(8_in, 90, true);
  chassis.pid_wait();
  pros::delay(250);
  chassis.pid_drive_set(-6_in, 80, true);
  chassis.pid_wait();
  intake_piston.set_value(true);
  chassis.pid_drive_set(6_in, 80, true);
  chassis.pid_wait();
  pros::delay(550);
  intake_piston.set_value(false);
  
  //Sweep corner
  chassis.pid_drive_set(-7_in, 90, true);
  chassis.pid_wait_quick();
  doinker_right.set_value(true);
  chassis.pid_turn_behavior_set(longest);
  chassis.pid_turn_set(140, 90, true);
  chassis.pid_wait_quick();
  disableAutoClamp();
  doinker_right.set_value(false);

  //Drop mogo in corner and set up for steal
  chassis.pid_drive_set(-10_in, 90, true);
  chassis.pid_wait_quick_chain();
  Intake.move(0);
  chassis.pid_drive_set(26_in, 127, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_behavior_set(shortest);
  chassis.pid_turn_set(4, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, 127, true);
  chassis.pid_wait_quick_chain();
}

void QualBlueNegative(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, -30_deg); 
  enableAutoClamp();
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  chassis.pid_odom_set(4_in, 75);
  chassis.pid_wait();
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  // Drive to mogo and clamp
  chassis.pid_odom_set(-38_in, 90, true);
  chassis.pid_wait_until(-20_in);
  chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  armPID.target_set(HOME);

  //Turn to 4 stack and start intake
  chassis.pid_turn_set(-185, 90, true);
  chassis.pid_wait();
  intakeWithSort();

  //Swing into 4 stack and intake 2 rings
  chassis.pid_drive_set(16_in, 127, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, -148, 100, 30);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, 60, true);
  chassis.pid_wait();
  pros::delay(450);
  
  //Back up and turn towards 2 stack
  chassis.pid_odom_set(-8_in, 90, true);
  chassis.pid_wait();
  Intake.move(0);
  chassis.pid_turn_set(-58, 80);
  chassis.pid_wait_quick_chain();
  
  //Intake 2 stack ring
  chassis.pid_drive_set(42_in, 75);
  intakeWithSort();
  chassis.pid_wait();
  
  //Turn towards corner and get bottom ring
  chassis.pid_turn_set(-102, 90);
  chassis.pid_wait();
  intakeWithSort();
  chassis.pid_odom_set(18_in, 80, true);
  chassis.pid_wait_quick_chain();
  pros::delay(200);

  //Back up and turn towards alliance 2 stack
  chassis.pid_odom_set(-20_in, 45, true);
  chassis.pid_wait_until(-8_in);
  chassis.pid_speed_max_set(80);
  chassis.pid_wait_quick();
  doinker_left.set_value(true);
  chassis.pid_turn_set(33, 90, true);
  chassis.pid_wait();
  doinker_left.set_value(false);

  //Go to 2 stack and intake top ring
  intakeWithSort();
  intake_piston.set_value(true);
  chassis.pid_drive_set(38_in, 127, true);
  chassis.pid_wait_until(20_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_quick();

  //Touch ladder
  intake_piston.set_value(false);
  chassis.pid_odom_set(-22_in, 45, true);
  chassis.pid_wait_until(-8_in);
  chassis.pid_speed_max_set(110);
  chassis.pid_wait();
  armPID.target_set(SCORING + 100);
  chassis.pid_turn_set(59, 90);
  chassis.pid_wait();
  chassis.pid_odom_set(25_in, 90);
  chassis.pid_wait();
}

void QualRedNegative(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, 30_deg); 
  enableAutoClamp();
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  chassis.pid_odom_set(4_in, 75);
  chassis.pid_wait();
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  // Drive to mogo and clamp
  chassis.pid_odom_set(-38_in, 90, true);
  chassis.pid_wait_until(-20_in);
  chassis.pid_speed_max_set(60);
  chassis.pid_wait();
  armPID.target_set(HOME);

  //Turn to 4 stack and start intake
  chassis.pid_turn_set(185, 90, true);
  chassis.pid_wait();
  intakeWithSort();

  //Swing into 4 stack and intake 2 rings
  chassis.pid_drive_set(15_in, 127, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 148, 100, 30);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, 60, true);
  chassis.pid_wait();
  pros::delay(450);
  
  //Back up and turn towards 2 stack
  chassis.pid_odom_set(-8_in, 90, true);
  chassis.pid_wait();
  Intake.move(0);
  chassis.pid_turn_set(58, 80);
  chassis.pid_wait_quick_chain();
  
  //Intake 2 stack ring
  chassis.pid_drive_set(42_in, 75);
  intakeWithSort();
  chassis.pid_wait();
  
  //Turn towards corner and get bottom ring
  chassis.pid_turn_set(108, 90);
  chassis.pid_wait();
  intakeWithSort();
  chassis.pid_odom_set(18_in, 80, true);
  chassis.pid_wait_quick_chain();
  pros::delay(200);

  //Back up and turn towards alliance 2 stack
  chassis.pid_odom_set(-20_in, 45, true);
  chassis.pid_wait_until(-8_in);
  chassis.pid_speed_max_set(80);
  chassis.pid_wait_quick();
  doinker_left.set_value(true);
  chassis.pid_turn_set(-33, 90, true);
  chassis.pid_wait();
  doinker_left.set_value(false);

  //Go to 2 stack and intake top ring
  intakeWithSort();
  intake_piston.set_value(true);
  chassis.pid_drive_set(38_in, 127, true);
  chassis.pid_wait_until(20_in);
  chassis.pid_speed_max_set(30);
  chassis.pid_wait_quick();

  //Touch ladder
  intake_piston.set_value(false);
  chassis.pid_odom_set(-22_in, 45, true);
  chassis.pid_wait_until(-8_in);
  chassis.pid_speed_max_set(110);
  chassis.pid_wait();
  armPID.target_set(SCORE_ALLIANCE + 100);
  chassis.pid_turn_set(-70, 90);
  chassis.pid_wait();
  chassis.pid_odom_set(25_in, 90);
  chassis.pid_wait();
}

void BluePositiveAWP(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, 30_deg); 
  enableAutoClamp();
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  chassis.pid_odom_set(4_in, 75);
  chassis.pid_wait();
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  // Drive to mogo and clamp
  chassis.pid_odom_set(-38_in, 90, true);
  chassis.pid_wait_until(-16_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait();
  armPID.target_set(HOME);

  //Turn to alliance 2 stack
  chassis.pid_turn_set(14, 90, true);
  chassis.pid_wait();

  //Get top ring from 2 stack
  intakeWithSort();
  intake_piston.set_value(true);
  chassis.pid_odom_set(32_in, 110, true);
  chassis.pid_wait();
  pros::delay(200);
  
  //Go backwards and turn towards center rings
  chassis.pid_odom_set(-26_in, 90, true);
  intake_piston.set_value(false);
  chassis.pid_wait();
  chassis.pid_turn_set(-104, 90, true);
  chassis.pid_wait();

  //Go to closest center ring and put down doinker
  chassis.pid_odom_set(14_in, 90, true);
  chassis.pid_wait_until(8_in);
  Intake.move(-5);
  chassis.pid_wait();
  doinker_right.set_value(true);

  //Go back with doinker down, put up doinker, turn, and intake center ring
  chassis.pid_odom_set(-31_in, 110);
  chassis.pid_wait_quick();
  doinker_right.set_value(false);
  pros::delay(90);
  chassis.pid_turn_set(-83, 90);
  chassis.pid_wait_quick();
  intakeWithSort();
  chassis.pid_odom_set(18_in, 90);
  chassis.pid_wait_quick();

  //Turn towards 2 stack
  chassis.pid_turn_set(-214, 90);
  chassis.pid_wait();
  pros::delay(500);
  disableAutoClamp();
  Intake.move(0);

  //Go to 2 stack and intake ring
  chassis.pid_odom_set(30_in, 110);
  chassis.pid_wait_until(8_in);
  intakeWithSort();
  chassis.pid_wait();

  //Stop intake about halfway up and drop mogo while turning to rush mogo
  Intake.move(10);

  //Go backwards to rush mogo and clamp
  chassis.pid_turn_set(-303, 90);
  chassis.pid_wait_quick();
  enableAutoClamp();
  chassis.pid_odom_set(-17.5_in, 75);
  chassis.pid_wait_quick();

  //Score 2 stack ring
  if(isGoalInClamp())
    intakeWithSort();
  else
    Intake.move(0);

  //Touch ladder
  armPID.target_set(SCORING + 100);
  chassis.pid_swing_set(ez::RIGHT_SWING, -85, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(12_in, 90);
  chassis.pid_wait();
}

void RedPositiveAWP(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, -30_deg); 
  enableAutoClamp();
  
  //Score alliance
  armPID.target_set(LOADING);
  Intake.move(127);
  pros::delay(600);
  Intake.move(-5);
  chassis.pid_odom_set(4_in, 75);
  chassis.pid_wait();
  armPID.target_set(SCORE_ALLIANCE);
  pros::delay(350);

  // Drive to mogo and clamp
  chassis.pid_odom_set(-38_in, 90, true);
  chassis.pid_wait_until(-16_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait();
  armPID.target_set(HOME);

  //Turn to alliance 2 stack
  chassis.pid_turn_set(-16, 90, true);
  chassis.pid_wait();

  //Get top ring from 2 stack
  intakeWithSort();
  intake_piston.set_value(true);
  chassis.pid_odom_set(30_in, 110, true);
  chassis.pid_wait();
  pros::delay(200);
  
  //Go backwards and turn towards center rings
  chassis.pid_odom_set(-26_in, 90, true);
  intake_piston.set_value(false);
  chassis.pid_wait();
  chassis.pid_turn_set(98, 90, true);
  chassis.pid_wait();

  //Go to closest center ring and put down doinker
  chassis.pid_odom_set(14_in, 90, true);
  chassis.pid_wait_until(8_in);
  Intake.move(-5);
  chassis.pid_wait();
  doinker_left.set_value(true);

  //Go back with doinker down, put up doinker, turn, and intake center ring
  chassis.pid_odom_set(-31_in, 110);
  chassis.pid_wait_quick();
  doinker_left.set_value(false);
  pros::delay(90);
  chassis.pid_turn_set(83, 90);
  chassis.pid_wait_quick();
  intakeWithSort();
  chassis.pid_odom_set(17_in, 90);
  chassis.pid_wait_quick();

  //Turn towards 2 stack
  chassis.pid_turn_set(214, 90);
  chassis.pid_wait();
  pros::delay(500);
  disableAutoClamp();
  Intake.move(0);

  //Go to 2 stack and intake ring
  chassis.pid_odom_set(30_in, 110);
  chassis.pid_wait_until(8_in);
  intakeWithSort();
  chassis.pid_wait();

  //Stop intake about halfway up and drop mogo while turning to rush mogo
  Intake.move(10);

  //Go backwards to rush mogo and clamp
  chassis.pid_turn_set(303, 90);
  chassis.pid_wait_quick();
  enableAutoClamp();
  chassis.pid_odom_set(-17.5_in, 75);
  chassis.pid_wait_quick();

  //Score 2 stack ring
  if(isGoalInClamp())
    intakeWithSort();
  else
    Intake.move(0);

  //Touch ladder 
  armPID.target_set(SCORING + 100);
  chassis.pid_swing_set(ez::LEFT_SWING, 85, 95, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(12_in, 90);
  chassis.pid_wait();
}

void Positive3Blue(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, -30_deg); 
  enableAutoClamp();

  // Drive to mogo and clamp
  chassis.pid_drive_set(-38_in, 90, true);
  chassis.pid_wait_until(-18_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait_quick();
  armPID.target_set(HOME);

  //Turn towards center rings
  intakeWithSort();
  chassis.pid_turn_set(-123, 85, true);
  chassis.pid_wait_quick();

  //Drive to center rings
  chassis.pid_drive_set(13_in, 90, true);
  chassis.pid_wait();
  
  //Turn towards red rings and put down doinker
  doinker_left.set_value(true);
  Intake.move(0);

  //Move back with red center rings
  chassis.pid_drive_set(-24_in, 110, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, -184, 100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-16_in, 120, true);
  chassis.pid_wait_quick();

  //Put up doinker and intake 2 rings
  doinker_left.set_value(false);
  pros::delay(200);
  intakeWithSort();

  //Turn around to 2 stack
  chassis.pid_turn_set(-114, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, -275, 100, 5);
  chassis.pid_wait_quick_chain();

  //Go to 2 stack and intake ring
  chassis.pid_drive_set(21_in, 75, true);
  chassis.pid_wait_quick();
  pros::delay(600);

  //Turn to drop mogo
  chassis.pid_turn_behavior_set(longest);
  chassis.pid_turn_set(-182, 110, true);
  chassis.pid_wait_until(-145);
  disableAutoClamp();
  chassis.pid_wait();
  
  //Turn to get ready for 3rd goal steal
  chassis.pid_drive_set(4_in, 80, true);
  chassis.pid_wait();
  chassis.pid_turn_set(-6, 110, true);
  chassis.pid_wait();
  enableAutoClamp();
  chassis.pid_drive_set(-4_in, 80, true);
  chassis.pid_wait();
}

void Positive3Red(){
  chassis.pid_odom_behavior_set(ez::shortest);

  //Change this to whatever it needs to be
  chassis.odom_xyt_set(0_in, 0_in, 30_deg); 
  enableAutoClamp();

  // Drive to mogo and clamp
  chassis.pid_drive_set(-38_in, 90, true);
  chassis.pid_wait_until(-18_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait_quick();
  armPID.target_set(HOME);

  //Turn towards center rings
  intakeWithSort();
  chassis.pid_turn_set(129, 85, true);
  chassis.pid_wait_quick();

  //Drive to center rings
  chassis.pid_drive_set(15.5_in, 90, true);
  chassis.pid_wait_until(6_in);
  Intake.move(0);
  chassis.pid_wait();
  
  //Turn towards red rings and put down doinker
  doinker_right.set_value(true);

  //Move back with red center rings
  chassis.pid_drive_set(-21_in, 110, true);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, 184, 100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-16_in, 120, true);
  chassis.pid_wait_quick();

  //Put up doinker and intake 2 rings
  doinker_right.set_value(false);
  pros::delay(200);
  intakeWithSort();

  //Turn around to 2 stack
  chassis.pid_turn_set(114, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 275, 100, 5);
  chassis.pid_wait_quick_chain();

  //Go to 2 stack and intake ring
  chassis.pid_drive_set(20_in, 75, true);
  chassis.pid_wait_quick();
  pros::delay(500);

  //Turn to drop mogo
  chassis.pid_turn_behavior_set(longest);
  chassis.pid_turn_set(182, 110, true);
  chassis.pid_wait_until(145);
  disableAutoClamp();
  chassis.pid_wait();
  
  //Turn to get ready for 3rd goal steal
  chassis.pid_drive_set(4_in, 80, true);
  chassis.pid_wait();
  chassis.pid_turn_set(4, 110, true);
  chassis.pid_wait();
  enableAutoClamp();
  chassis.pid_drive_set(-8_in, 80, true);
  chassis.pid_wait();
}

void test(){
  chassis.pid_odom_behavior_set(ez::shortest);

  chassis.pid_odom_set({
    {{0_in, 24_in, 45_deg}, fwd, 110},
    {{24_in, 24_in, -90_deg}, fwd, 110}},
    true);
  chassis.pid_wait();
}
//   if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
//   if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
//   if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
//   if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
// }
