#pragma once

#include <array>
#include <cstdlib>
#include <iostream>
#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

extern Drive chassis;

//Drive Motors, all 600 rpm blue carts
inline::pros::Motor FrontL(10, pros::MotorGearset::blue);
inline::pros::Motor MidL(-9, pros::MotorGearset::blue);
inline::pros::Motor BackL(-20, pros::MotorGearset::blue);
inline::pros::Motor FrontR(-1, pros::MotorGearset::blue);
inline::pros::Motor MidR(2, pros::MotorGearset::blue);
inline::pros::Motor BackR(11, pros::MotorGearset::blue);

// Clamp : port A | Left Doinker : port B | Right Doinker : port C | Intake Piston : port D
inline::pros::adi::DigitalOut clamp_piston(1);
inline::pros::adi::DigitalOut doinker_left(2);
inline::pros::adi::DigitalOut doinker_right(3);
inline::pros::adi::DigitalOut intake_piston(4);

//                                                   <<Piston variables>>
inline static bool clampPiston  = false;  //--------> toggle for mogo clamp
inline static bool doinkPistonL = false;  //--------> toggle for left doinker
inline static bool doinkPistonR = false;  //--------> toggle for right doinker
inline static bool intakePiston = false;  //--------> toggle for intake piston

// Distance sensor on port 6
inline::pros::Distance clampSensor(6);
inline int  distToSensor  = 5;            //--------> how far away the mogo has to be to get clamped

//Arm Motor, 2 200rpm half watt motors
inline::pros::Motor ArmL(15, pros::MotorGearset::green);
inline::pros::Motor ArmR(-18, pros::MotorGearset::green);
// Arm PID initialization
inline ez::PID armPID{.6, 0, 0};

// Moves both motors for the arm based on input variable
inline void set_arm(int input) {
  ArmL.move(input);
  ArmR.move(input);
}

// Waits for the arm to reach the target position, use in auton if you want (its not used)
inline void arm_wait() {
  while (armPID.exit_condition({ArmL, ArmR}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

// Intake Motors, 1 200 rpm green cart for hooks
inline::pros::Motor Intake(21, pros::MotorGearset::green);

// Checks the intake temperature
inline float checkIntakeTemp() {
  return Intake.get_temperature();
}

// Checks if the intake is overheated; returns true if the temp is above 50C
inline bool isIntakeOverheated() {
  if(checkIntakeTemp() > 50)
    return true;
  else
    return false;
}

// Optical sensor on port 19
inline::pros::Optical optical(19);

// Optical states (for rings)
enum ColorState {
  RED,
  BLUE,
  NONE
};

// Team states
enum Team {
  RED_TEAM,
  BLUE_TEAM
};

//                                                   <<Color sort variables>>
inline ColorState currentRingColor = NONE;
inline Team  team = BLUE_TEAM;

inline int  blueLowerHue  = 220;   //---------------> hue values for the sensor to detect
inline int  blueHigherHue = 290;
inline int  redLowerHue   = 350;
inline int  redHigherHue  = 30;

inline int timeToTop = 90;        //----------------> for flicking the ring out of the top
inline int reverseDelay = 120;

inline bool isSorting     = false; //---------------> is the color sort active
inline bool sortOverride  = false; //---------------> toggle for color sort being on or off

// Toggles between sorting out red or blue rings
inline void toggleColorSort() {
  if(team == BLUE_TEAM)
    team = RED_TEAM;
  else
    team = BLUE_TEAM;
}

// Detects and returns the color of the ring that the optical detects, if there is any
inline ColorState getCurrentRingColor() {
  float hue = optical.get_hue();
  float saturation = optical.get_saturation();
  // If there isn't something really close to the sensor, there isn't a ring
  if(optical.get_proximity() < 240) {
    return NONE;
  } else if ((hue > blueLowerHue && hue < blueHigherHue)) {
    return BLUE;
  } else if ((hue > redLowerHue || hue < redHigherHue)) {
    return RED;
  } 
}

// Decides whether or not to sort out the detected ring; Sorts out if the team color and ring color don't match
inline bool shouldSort(ColorState) {
  if/*--*/(team == RED_TEAM && currentRingColor == BLUE)
    return true;
  else if (team == BLUE_TEAM && currentRingColor == RED)
    return true;
  else
    return false;
}

// Flicks the ring out of the top of the intake
inline void moveRing() {
  float pastVoltage = Intake.get_voltage();

  pros::delay(timeToTop);
  Intake.move(-127);
  pros::delay(reverseDelay);
  Intake.move(pastVoltage);
}

// Simplified function that takes in the detected color and decides whether or not to sort
inline static void sortRing(ColorState color){
  bool sortOrNah = shouldSort(color);

  if(sortOrNah) {
    isSorting = true;
    moveRing();
    isSorting = false;
  } else {
    isSorting = false;
  }
}
