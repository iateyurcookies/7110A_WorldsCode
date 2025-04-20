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

inline::pros::Rotation intakeRotation(-16);

// Optical sensor on port 19
inline::pros::Optical optical(19);

// Distance sensor on port 11 and port **
inline::pros::Distance clampSensor(6);
inline int  distToSensor  = 5;            //---------------> how far away the mogo has to be to get clamped

//                                                   <<Piston variables>>
inline static bool clampPiston  = false;  //---------------> toggle for mogo clamp
inline static bool doinkPiston  = false;  //---------------> toggle for doinker
inline static bool intakePiston = false;  //---------------> toggle for intake piston
inline static bool doinkClamp   = true;   //---------------> toggle for doinker clamp

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

// Optical states
enum ColorState {
  RED,
  BLUE,
  NONE
};

enum Team {
  RED_TEAM,
  BLUE_TEAM
};

//                                                   <<Color sort variables>>
inline ColorState currentRingColor = NONE;
inline Team  team = BLUE_TEAM;

inline int  blueLowerHue  = 200;   //---------------> hue values for the sensor to detect
inline int  blueHigherHue = 240;
inline int  redLowerHue   = 1;
inline int  redHigherHue  = 10;

inline bool isSorting     = false; //---------------> is the color sort active
inline bool sortOverride  = false; //---------------> toggle for color sort being on or off

inline void toggleColorSort() {
  if(team == BLUE_TEAM)
    team = RED_TEAM;
  else
    team = BLUE_TEAM;
}

inline ColorState getCurrentRingColor() {
  float hue = optical.get_hue();
  if/*---*/ ((hue > blueLowerHue && hue < blueHigherHue)) {
    return BLUE;
  } else if ((hue > redLowerHue && hue < redHigherHue)) {
    return RED;
  } else {
    return NONE;
  }
}

inline bool shouldSort(ColorState) {
  if/*--*/(team == RED_TEAM && currentRingColor == BLUE)
    return true;
  else if (team == BLUE_TEAM && currentRingColor == RED)
    return true;
  else
    return false;
}

inline void moveRing(float target) {
  float currentIntakePosition = intakeRotation.get_position();

  while(abs(currentIntakePosition - target) > 100) {
    Intake.move(127);
  }
  Intake.move(-127);
  pros::delay(80);
  Intake.move(0);
}

inline static void sortRing(ColorState color){
  bool sortOrNah = shouldSort(color);

  if(sortOrNah) {
    isSorting = true;
    moveRing(8000);
    isSorting = false;
  } else {
    isSorting = false;
  }
}

