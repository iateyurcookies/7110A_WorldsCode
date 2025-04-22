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

inline int  blueLowerHue  = 220;   //---------------> hue values for the sensor to detect
inline int  blueHigherHue = 290;
inline int  redLowerHue   = 350;
inline int  redHigherHue  = 30;

inline int timeToTop = 90;        //---------------> for flicking the ring out of the top
inline int reverseDelay = 120;

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
  float saturation = optical.get_saturation();

  // Adjust these hue values to detect the ring more accurately
  if/*---*/ ((hue > blueLowerHue && hue < blueHigherHue) && saturation > .40) {
    return BLUE;
  } else if ((hue > redLowerHue || hue < redHigherHue) && saturation > .40) {
    return RED;
  } 
  // If there isnt something really close to the sensor, there is not a ring
  if(optical.get_proximity() < 240) {
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

inline void moveRing() {
  float pastVoltage = Intake.get_voltage();

  pros::delay(timeToTop);
  Intake.move(-127);
  pros::delay(reverseDelay);
  Intake.move(pastVoltage);
}

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
