/**
 * @file sim/pigpiod_if2.h
 * @brief No-op stand-in for the pigpio daemon client library.
 *
 * This header deliberately shadows the real <pigpiod_if2.h>. When the package
 * is configured with -DTWO_TOWERS_SIM_GPIO=ON, this directory is placed ahead
 * of the system include path, so Turret.cpp and ServoController.cpp compile
 * unchanged against these stubs instead of the real library.
 *
 * The point is to let the control stack build and run on a machine with no
 * Raspberry Pi attached -- CI runners, laptops, and unit tests. Servo commands
 * become no-ops; everything above the GPIO boundary behaves identically.
 *
 * This is NOT a servo simulator. It does not model travel time, slew rate, or
 * the servo's internal position loop. It exists so that code which does not
 * care about GPIO can be built and tested without it.
 *
 * Only the three pigpio entry points this project actually uses are stubbed.
 * If you reach for a fourth, add it here or CI will tell you immediately.
 */

#ifndef TWO_TOWERS_SIM_PIGPIOD_IF2_H
#define TWO_TOWERS_SIM_PIGPIOD_IF2_H

/// Pretend to connect to a local pigpiod. Returns a valid (non-negative) handle.
inline int pigpio_start(const char* /*addr*/, const char* /*port*/) { return 0; }

/// Pretend to disconnect.
inline void pigpio_stop(int /*pi*/) {}

/// Accept and discard a servo pulse width command.
inline int set_servo_pulsewidth(int /*pi*/, unsigned /*user_gpio*/,
                                unsigned /*pulsewidth*/) { return 0; }

#endif  // TWO_TOWERS_SIM_PIGPIOD_IF2_H
