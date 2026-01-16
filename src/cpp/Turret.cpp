/**
 * @file Turret.cpp
 * @brief High-Level Pan/Tilt Turret Controller Implementation
 *
 * Implements the Turret class for coordinated pan/tilt servo control
 * via the pigpio daemon.
 *
 * @author Tate Lloyd <tate.lloyd@yale.edu>
 * @license MIT
 */

#include <pigpiod_if2.h>
#include <iostream>
#include <stdexcept>

#include "Turret.hpp"


//=============================================================================
// CONSTRUCTOR / DESTRUCTOR
//=============================================================================

Turret::Turret(int pan_pin, int tilt_pin)
    : pi(pigpio_start(nullptr, nullptr)),  // Connect to local daemon
      pan(pan_pin, pi),                     // Initialize pan servo
      tilt(tilt_pin, pi)                    // Initialize tilt servo
{
    // Verify daemon connection succeeded
    if (pi < 0) {
        throw std::runtime_error(
            "Failed to connect to pigpio daemon. "
            "Ensure pigpiod is running: sudo systemctl start pigpiod"
        );
    }

    std::cout << "[Turret] Connected to pigpio daemon (handle: " << pi << ")" << std::endl;
    std::cout << "[Turret] Pan servo on GPIO " << pan_pin << std::endl;
    std::cout << "[Turret] Tilt servo on GPIO " << tilt_pin << std::endl;
}


Turret::~Turret() {
    std::cout << "[Turret] Shutting down..." << std::endl;

    // ServoController destructors will center each servo automatically
    // Then disconnect from daemon
    pigpio_stop(pi);

    std::cout << "[Turret] Disconnected from pigpio daemon" << std::endl;
}


//=============================================================================
// ANGLE-BASED CONTROL
//=============================================================================

void Turret::setPanAngle(double degrees) {
    pan.setAngle(degrees);
}


void Turret::setTiltAngle(double degrees) {
    tilt.setAngle(degrees);
}


//=============================================================================
// PULSE WIDTH CONTROL
//=============================================================================

void Turret::setPanPulseWidth(int microseconds) {
    pan.setPulseWidth(microseconds);
}


void Turret::setTiltPulseWidth(int microseconds) {
    tilt.setPulseWidth(microseconds);
}


//=============================================================================
// STATE QUERIES
//=============================================================================

double Turret::getPanAngle() const {
    return pan.getCurrentAngle();
}


double Turret::getTiltAngle() const {
    return tilt.getCurrentAngle();
}


//=============================================================================
// UTILITY
//=============================================================================

void Turret::centerAll() {
    pan.center();           // Pan to 90° (center)
    tilt.setAngle(30);      // Tilt to 30° (slight downward default)
}
