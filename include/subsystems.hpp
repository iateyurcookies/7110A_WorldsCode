#pragma once

#include <array>
#include <iostream>
#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

//Drive Motors, all 600 rpm blue carts
inline::pros::Motor FrontL(10, pros::MotorGearset::blue);
inline::pros::Motor MidL(9, pros::MotorGearset::blue);
inline::pros::Motor BackL(8, pros::MotorGearset::blue);
inline::pros::Motor FrontR(-7, pros::MotorGearset::blue);
inline::pros::Motor MidR(-6, pros::MotorGearset::blue);
inline::pros::Motor BackR(-5, pros::MotorGearset::blue);

// Intake Motors, 1 600 rpm blue cart for hooks, 1 200 rpm half watt for flex wheels
inline::pros::Motor Intake(21, pros::MotorGearset::blue);

//Arm Motor, 1 200rpm half watt motor
inline::pros::Motor ArmL(-99, pros::MotorGearset::green);
inline::pros::Motor ArmR(-99, pros::MotorGearset::green);

// Rotation sensor for arm on port 19
inline::pros::Rotation ArmSensor(99); 

// Arm PID stuff
inline ez::PID armPID{20.0, 0, 100.0};

inline void set_arm(int input) {
  ArmL.move(input);
  ArmR.move(input);
}

inline void arm_wait() {
  while (armPID.exit_condition({ArmL, ArmR}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

// Clamp, Doinker Piston, Doinker Clamp, & Intake Piston
// Clamp : port A | Doinker : port B | Doinker Clamp : port C | Intake Piston : port D
inline::pros::adi::DigitalOut clamp_piston(1);
inline::pros::adi::DigitalOut doinker_piston(2);
inline::pros::adi::DigitalOut intake_piston(3);
inline::pros::adi::DigitalOut doinker_clamp(4);

// Optical sensor on port 20
inline::pros::Optical optical(99);

// Distance sensor on port 11 and port **
inline::pros::Distance distanceSensor(99);
inline::pros::Distance clampSensor(99);



