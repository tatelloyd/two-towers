/**
 * @file Turret.hpp
 * @brief High-Level Pan/Tilt Turret Controller Interface
 *
 * Provides a unified interface for controlling a two-axis pan/tilt turret
 * with SG90 servo motors via pigpio daemon. Manages hardware initialization,
 * resource cleanup, and coordinate angle/pulse width commands.
 *
 * @section HardwareConfig Hardware Configuration
 *
 *     ┌─────────────────────────────┐
 *     │         TURRET              │
 *     │                             │
 *     │   Pan Servo (GPIO 17)       │
 *     │   └─ Horizontal rotation    │
 *     │   └─ Range: 0° - 180°       │
 *     │                             │
 *     │   Tilt Servo (GPIO 27)      │
 *     │   └─ Vertical rotation      │
 *     │   └─ Range: 0° - 180°       │
 *     │                             │
 *     └─────────────────────────────┘
 *
 * @section ResourceManagement Resource Management
 *
 * Uses RAII pattern:
 *   - Constructor connects to pigpio daemon
 *   - Destructor centers servos and disconnects
 *   - Single daemon handle shared between both servos
 *
 * @section Dependencies Dependencies
 *
 *   - pigpiod daemon must be running (sudo systemctl start pigpiod)
 *   - ServoController class for PWM abstraction
 *
 * @author Tate Lloyd <tate.lloyd@yale.edu>
 * @license MIT
 */

#ifndef TURRET_HPP
#define TURRET_HPP

#include "ServoController.hpp"

/**
 * @class Turret
 * @brief High-level controller for pan/tilt turret assembly
 *
 * Manages two servo motors (pan and tilt) as a coordinated unit.
 * Provides angle-based control interface and automatic centering
 * on shutdown for safety.
 *
 * Example Usage:
 * @code
 *     Turret turret(17, 27);      // Pan on GPIO 17, tilt on GPIO 27
 *     turret.setPanAngle(90.0);   // Center horizontally
 *     turret.setTiltAngle(45.0);  // 45 degrees up
 *     // Destructor auto-centers on exit
 * @endcode
 */
class Turret {
public:
    //=========================================================================
    // LIFECYCLE
    //=========================================================================

    /**
     * @brief Construct turret and connect to pigpio daemon
     *
     * Initializes both servo controllers and connects to the pigpio daemon.
     * Throws exception if daemon connection fails.
     *
     * @param pan_pin GPIO pin number for pan servo (horizontal)
     * @param tilt_pin GPIO pin number for tilt servo (vertical)
     *
     * @throws std::runtime_error if pigpio daemon connection fails
     *
     * @note Requires pigpiod daemon running: sudo systemctl start pigpiod
     */
    Turret(int pan_pin, int tilt_pin);

    /**
     * @brief Destructor - centers servos and disconnects from daemon
     *
     * Automatically returns turret to safe center position before
     * disconnecting from the pigpio daemon.
     */
    ~Turret();

    //=========================================================================
    // ANGLE-BASED CONTROL
    //=========================================================================

    /**
     * @brief Set pan servo angle (horizontal rotation)
     * @param degrees Angle in degrees (0° = full left, 180° = full right)
     */
    void setPanAngle(double degrees);

    /**
     * @brief Set tilt servo angle (vertical rotation)
     * @param degrees Angle in degrees (0° = down, 180° = up)
     */
    void setTiltAngle(double degrees);

    //=========================================================================
    // PULSE WIDTH CONTROL (LOW-LEVEL)
    //=========================================================================

    /**
     * @brief Set pan servo pulse width directly
     * @param microseconds PWM pulse width (500-2500 us typical)
     */
    void setPanPulseWidth(int microseconds);

    /**
     * @brief Set tilt servo pulse width directly
     * @param microseconds PWM pulse width (500-2500 us typical)
     */
    void setTiltPulseWidth(int microseconds);

    //=========================================================================
    // STATE QUERIES
    //=========================================================================

    /**
     * @brief Get current pan servo angle
     * @return Current pan angle in degrees
     */
    double getPanAngle() const;

    /**
     * @brief Get current tilt servo angle
     * @return Current tilt angle in degrees
     */
    double getTiltAngle() const;

    //=========================================================================
    // UTILITY
    //=========================================================================

    /**
     * @brief Return turret to center position
     *
     * Centers pan servo (90°) and sets tilt to default down-angle (30°).
     * Called automatically by destructor.
     */
    void centerAll();

private:
    /// Handle to pigpio daemon connection (shared between servos)
    int pi;

    /// Pan (horizontal) servo controller
    ServoController pan;

    /// Tilt (vertical) servo controller
    ServoController tilt;
};

#endif // TURRET_HPP
