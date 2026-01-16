/**
 * @file ServoController.cpp
 * @brief PWM Servo Control Implementation
 *
 * Implements the ServoController class for hardware PWM servo control
 * via the pigpio daemon.
 *
 * @author Tate Lloyd <tate.lloyd@yale.edu>
 * @license MIT
 */

#include "ServoController.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <algorithm>


//=============================================================================
// CONSTRUCTOR / DESTRUCTOR
//=============================================================================

ServoController::ServoController(int pin_num, int pi_handle,
                                 int min_pw, int max_pw, int center_pw)
    : pi(pi_handle),
      pin(pin_num),
      min_pw(min_pw),
      max_pw(max_pw),
      center_pw(center_pw),
      current_pw(center_pw)
{
    // Initialize servo to center position
    set_servo_pulsewidth(pi, pin, center_pw);
    current_pw = center_pw;
}


ServoController::~ServoController() {
    // Return servo to center position before shutdown
    set_servo_pulsewidth(pi, pin, center_pw);

    // NOTE: We do NOT call pigpio_stop(pi) here.
    // The daemon handle is shared across multiple servos and is
    // managed by the parent Turret class.
}


//=============================================================================
// CONTROL INTERFACE
//=============================================================================

void ServoController::setAngle(double degrees) {
    int pw = angleToPulseWidth(degrees);
    setPulseWidth(pw);
}


void ServoController::setPulseWidth(int microseconds) {
    // Clamp to valid range to prevent servo damage
    int clamped_pw = clampPulseWidth(microseconds);

    // Command hardware
    set_servo_pulsewidth(pi, pin, clamped_pw);

    // Track current position for state queries
    current_pw = clamped_pw;
}


void ServoController::center() {
    setPulseWidth(center_pw);
}


//=============================================================================
// STATE QUERIES
//=============================================================================

double ServoController::getCurrentAngle() const {
    return pulseWidthToAngle(current_pw);
}


//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

int ServoController::clampPulseWidth(int pw) const {
    return std::clamp(pw, min_pw, max_pw);
}


int ServoController::angleToPulseWidth(double degrees) const {
    // Linear mapping: [0°, 180°] -> [min_pw, max_pw]
    //
    // Formula:
    //   normalized = degrees / 180.0     (0.0 to 1.0)
    //   pw = min_pw + normalized * (max_pw - min_pw)
    //
    // Example with default values (500, 2500):
    //   0°   -> 500 μs
    //   90°  -> 1500 μs
    //   180° -> 2500 μs

    double normalized = degrees / 180.0;
    return static_cast<int>(min_pw + normalized * (max_pw - min_pw));
}


double ServoController::pulseWidthToAngle(int pw) const {
    // Inverse of angleToPulseWidth()
    //
    // Formula:
    //   normalized = (pw - min_pw) / (max_pw - min_pw)
    //   degrees = normalized * 180.0

    double normalized = static_cast<double>(pw - min_pw) / (max_pw - min_pw);
    return normalized * 180.0;
}
