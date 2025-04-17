#pragma once

#include <array>
#include <iostream>
#include "EZ-Template/api.hpp"
#include "api.h"

//                                                   <<Piston variables>>
inline static bool clampPiston  = false;  //---------------> toggle for mogo clamp
inline static bool doinkPiston  = false;  //---------------> toggle for doinker
inline static bool intakePiston = false;  //---------------> toggle for intake piston
inline static bool doinkClamp   = true;   //---------------> toggle for doinker clamp

//                                                   <<Color sort variables>>
inline bool team          = false; //---------------> true = red    false = blue
inline bool ringCol       = false; //---------------> sensor detected color
inline bool isSorting     = false; //---------------> is the color sort active
inline int  blueLowerHue  = 200;   //---------------> hue values for the sensor to detect
inline int  blueHigherHue = 240;
inline int  redLowerHue   = 10;
inline int  redHigherHue  = 20;
inline int  distToSensor  = 5;    //---------------> how far away the ring has to be for the sensor to detect it
inline bool sortOverride  = false; //---------------> toggle for color sort being on or off

extern Drive chassis;

//Drive Motors, all 600 rpm blue carts
inline::pros::Motor FrontL(10, pros::MotorGearset::blue);
inline::pros::Motor MidL(-9, pros::MotorGearset::blue);
inline::pros::Motor BackL(-20, pros::MotorGearset::blue);
inline::pros::Motor FrontR(-1, pros::MotorGearset::blue);
inline::pros::Motor MidR(2, pros::MotorGearset::blue);
inline::pros::Motor BackR(11, pros::MotorGearset::blue);

// Intake Motors, 1 600 rpm blue cart for hooks, 1 200 rpm half watt for flex wheels
inline::pros::Motor Intake(21, pros::MotorGearset::green);

//Arm Motor, 1 200rpm half watt motor
inline::pros::Motor ArmL(15, pros::MotorGearset::green);
inline::pros::Motor ArmR(-18, pros::MotorGearset::green);


// Clamp, Doinker Piston, Doinker Clamp, & Intake Piston
// Clamp : port A | Doinker : port B | Doinker Clamp : port C | Intake Piston : port D
inline::pros::adi::DigitalOut clamp_piston(1);
inline::pros::adi::DigitalOut doinker_piston(2);
inline::pros::adi::DigitalOut intake_piston(3);
inline::pros::adi::DigitalOut doinker_clamp(4);

// Optical sensor on port 19
inline::pros::Optical optical(19);

// Distance sensor on port 11 and port **
inline::pros::Distance intakeDistance(13);
inline::pros::Distance clampSensor(6);

// Arm PID stuff
inline ez::PID armPID{.6, 0, 0};

inline void set_arm(int input) {
  ArmL.move(input);
  ArmR.move(input);
}

inline void arm_wait() {
  while (armPID.exit_condition({ArmL, ArmR}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

inline void toggleColorSort() {
  if(team == true)
    team = false;
  else if(team == false)
    team = true;
}

inline void getCurrentRingColor() {
  if/*---*/ ((optical.get_hue() > blueLowerHue || optical.get_hue() < blueHigherHue) 
  && optical.get_saturation() > 35 && intakeDistance.get_distance() < 50) {
    ringCol = false; //----------------------------> Blue
  } else if ((optical.get_hue() > redLowerHue || optical.get_hue() < redHigherHue)
  && optical.get_saturation() > 35 && intakeDistance.get_distance() < 50) {
    ringCol = true; //-----------------------------> Red
  }
}

inline static bool isOppositeColor() {
  if(team != ringCol)
    return true;
  else
    return false;
}

inline static void flickRing(){
  isSorting = true;
  Intake.move_relative(100, 400);
  while(Intake.get_position() < Intake.get_target_position()) {
    pros::delay(2);
  }
  Intake.move(0);
  pros::delay(150);
  isSorting = false;
}

