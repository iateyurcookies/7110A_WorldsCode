#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

enum class ArmTarget {
  HOME,
  GRAB_RING,
  RETURN_GRAB,
  SCORE_RING,
  TIPPING_GOAL,
  SCORE_ALLIANCE
};

// IMU/Inertial Sensor
inline::pros::IMU iMu(1);

//Drive Motors, all 600 rpm blue carts
inline::pros::Motor FrontL(10, pros::MotorGearset::blue);
inline::pros::Motor MidL(9, pros::MotorGearset::blue);
inline::pros::Motor BackL(8, pros::MotorGearset::blue);
inline::pros::Motor FrontR(-7, pros::MotorGearset::blue);
inline::pros::Motor MidR(-6, pros::MotorGearset::blue);
inline::pros::Motor BackR(-5, pros::MotorGearset::blue);

// Intake Motors, 1 600 rpm blue cart for hooks, 1 200 rpm half watt for flex wheels
inline::pros::Motor Intake(-4, pros::MotorGearset::blue);
inline::pros::Motor IntakeFlex(3, pros::MotorGearset::green);

//Arm Motor, 1 200rpm half watt motor
inline::pros::Motor ArmL(-99, pros::MotorGearset::green);
inline::pros::Motor ArmR(-99, pros::MotorGearset::green);

// Rotation sensor for arm on port 19
inline::pros::Rotation ArmSensor(19);

inline static float kP = 0.45;             // PID constants for arm
inline static float kI = 0.0;
inline static float kD = 0.1; 

// Arm PID stuff
inline ez::PID armPID{kP, kI, kD};

inline void arm_wait() {
  while (armPID.exit_condition({ArmL, ArmR}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

inline float getArmTargetPosition(ArmTarget target) {
  switch (target) {
    case ArmTarget::HOME:          return 5;
    case ArmTarget::GRAB_RING:     return 11;
    case ArmTarget::RETURN_GRAB:   return 11;
    case ArmTarget::SCORE_RING:    return 138;
    case ArmTarget::SCORE_ALLIANCE:return 138;
    case ArmTarget::TIPPING_GOAL:  return 268;
    default: return 0;
  }
}

// Clamp, Doinker Piston, Doinker Clamp, & Intake Piston
// Clamp : port A | Doinker : port B | Doinker Clamp : port C | Intake Piston : port D
inline::pros::adi::DigitalOut clamp_piston(1);
inline::pros::adi::DigitalOut doinker_piston(2);
inline::pros::adi::DigitalOut intake_piston(3);
inline::pros::adi::DigitalOut doinker_clamp(4);

// Optical sensor on port 20
inline::pros::Optical optical(18);

// Distance sensor on port 11 and port **
inline::pros::Distance distanceSensor(11);
inline::pros::Distance clampSensor(99);



