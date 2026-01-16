/**
 * @file ServoController.hpp
 * @brief PWM Servo Control Abstraction Layer
 *
 * Provides a hardware abstraction for PWM servo control via the pigpio daemon.
 * Handles angle-to-pulse-width conversion, clamping, and resource management.
 *
 * @section PWM PWM Specifications
 *
 * Standard hobby servo PWM timing:
 *
 *     ←───────── 20ms Period (50 Hz) ─────────→
 *     ┌────┐
 *     │    │
 *     │    └─────────────────────────────────────
 *     ←────→
 *     Pulse Width:
 *       500 μs  = 0° (minimum)
 *       1500 μs = 90° (center)
 *       2500 μs = 180° (maximum)
 *
 * @section SafetyFeatures Safety Features
 *
 *   - Pulse width clamping prevents damage from out-of-range values
 *   - Auto-centering on destruction prevents erratic final position
 *   - Copy operations deleted to prevent duplicate hardware access
 *
 * @author Tate Lloyd <tate.lloyd@yale.edu>
 * @license MIT
 */

#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

/**
 * @class ServoController
 * @brief Low-level PWM servo control via pigpio daemon
 *
 * Provides both angle-based and direct pulse-width control interfaces
 * for hobby servo motors. Manages a single GPIO pin and tracks current
 * position for state queries.
 *
 * @note This class does NOT own the pigpio daemon handle. The daemon
 * connection lifecycle is managed by the parent Turret class.
 *
 * Example Usage:
 * @code
 *     int pi = pigpio_start(nullptr, nullptr);
 *     ServoController servo(17, pi);  // GPIO 17
 *     servo.setAngle(90.0);           // Center position
 *     servo.setAngle(45.0);           // 45 degrees
 *     // Destructor auto-centers
 * @endcode
 */
class ServoController {
public:
    //=========================================================================
    // LIFECYCLE
    //=========================================================================

    /**
     * @brief Construct servo controller with custom PWM range
     *
     * Initializes servo to center position immediately.
     *
     * @param pin_num GPIO pin number for this servo
     * @param pi_handle Handle from pigpio_start() - NOT owned by this class
     * @param min_pw Minimum pulse width in microseconds (default: 500)
     * @param max_pw Maximum pulse width in microseconds (default: 2500)
     * @param center_pw Center position pulse width in microseconds (default: 1500)
     */
    ServoController(int pin_num, int pi_handle,
                    int min_pw = 500, int max_pw = 2500, int center_pw = 1500);

    /**
     * @brief Destructor - returns servo to center position
     *
     * @note Does NOT call pigpio_stop(). Daemon handle is shared and
     * managed by the parent Turret class.
     */
    ~ServoController();

    //=========================================================================
    // DELETED OPERATIONS (Prevent Hardware Conflicts)
    //=========================================================================

    /// Deleted copy constructor - prevents duplicate hardware access
    ServoController(const ServoController&) = delete;

    /// Deleted copy assignment - prevents duplicate hardware access
    ServoController& operator=(const ServoController&) = delete;

    //=========================================================================
    // CONTROL INTERFACE
    //=========================================================================

    /**
     * @brief Set servo position by angle
     *
     * Converts angle to pulse width and commands servo. Angle is
     * mapped linearly from [0°, 180°] to [min_pw, max_pw].
     *
     * @param degrees Target angle (0.0 - 180.0 degrees)
     */
    void setAngle(double degrees);

    /**
     * @brief Set servo position by pulse width directly
     *
     * Allows direct PWM control for calibration or non-standard servos.
     * Pulse width is clamped to configured min/max range.
     *
     * @param microseconds Pulse width in microseconds
     */
    void setPulseWidth(int microseconds);

    /**
     * @brief Return servo to center position
     *
     * Moves servo to the configured center_pw position (typically 90°).
     */
    void center();

    //=========================================================================
    // STATE QUERIES
    //=========================================================================

    /**
     * @brief Get current pulse width
     * @return Current commanded pulse width in microseconds
     */
    int getCurrentPulseWidth() const { return current_pw; }

    /**
     * @brief Get current angle
     * @return Current servo angle in degrees (computed from pulse width)
     */
    double getCurrentAngle() const;

private:
    //=========================================================================
    // MEMBER VARIABLES
    //=========================================================================

    int pi;          ///< pigpio daemon handle (NOT owned)
    int pin;         ///< GPIO pin number
    int min_pw;      ///< Minimum pulse width (microseconds)
    int max_pw;      ///< Maximum pulse width (microseconds)
    int center_pw;   ///< Center position pulse width (microseconds)
    int current_pw;  ///< Current commanded pulse width (for state tracking)

    //=========================================================================
    // HELPER FUNCTIONS
    //=========================================================================

    /**
     * @brief Clamp pulse width to valid range
     * @param pw Input pulse width
     * @return Pulse width clamped to [min_pw, max_pw]
     */
    int clampPulseWidth(int pw) const;

    /**
     * @brief Convert angle to pulse width
     *
     * Linear mapping: 0° -> min_pw, 180° -> max_pw
     *
     * @param degrees Angle in degrees
     * @return Corresponding pulse width in microseconds
     */
    int angleToPulseWidth(double degrees) const;

    /**
     * @brief Convert pulse width to angle
     *
     * Inverse of angleToPulseWidth().
     *
     * @param pw Pulse width in microseconds
     * @return Corresponding angle in degrees
     */
    double pulseWidthToAngle(int pw) const;
};

#endif // SERVO_CONTROLLER_HPP
