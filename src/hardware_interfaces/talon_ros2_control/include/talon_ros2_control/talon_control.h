#ifndef TALON_CONTROL_H
#define TALON_CONTROL_H

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

/**
 * @brief Per-joint motor configuration.
 *
 * Replaces the old hardcoded #defines so that each joint can have its own
 * encoder resolution, gear ratio, PID gains, etc.  Defaults match the
 * Pololu 37D gearmotors that were previously hardcoded.
 */
struct MotorConfig {
  double encoder_resolution = 64.0;    // Encoder counts per motor revolution
  double gear_ratio         = 102.08;  // Motor-to-output shaft gear ratio
  double distance_per_rev   = 0.0015;  // Meters per output revolution (prismatic only)
  bool   inverted           = true;    // Motor output direction
  bool   sensor_phase       = true;    // Encoder phase relative to motor
  double kP                 = 0.25;    // Proportional gain (slot 0)
  double kI                 = 0.0;     // Integral gain (slot 0)
  double kD                 = 0.0;     // Derivative gain (slot 0)
  double kF                 = 0.0;     // Feed-forward gain (slot 0)
  int    config_timeout_ms  = 100;     // Timeout for TalonSRX config calls
};

// Motor initialization
void initMotor(TalonSRX * motor, const MotorConfig & config, const std::string & can_interface);

// Conversions (config-aware)
double convertRevtoTalonUnits(double rev, const MotorConfig & config);
double convertTalonUnitstoRev(double counts, const MotorConfig & config);
double convertRevToDistance(double rev, const MotorConfig & config);
double convertDistanceToRev(double distance, const MotorConfig & config);

// Getters — position
float getPositionTalonUnits(TalonSRX * motor);
float getPositionRevolutions(TalonSRX * motor, const MotorConfig & config);
float getPositionDistance(TalonSRX * motor, const MotorConfig & config);
float getPositionRadians(TalonSRX * motor, const MotorConfig & config);

// Getters — velocity
float getVelocityRPM(TalonSRX * motor, const MotorConfig & config);
float getLinearVelocity(TalonSRX * motor, const MotorConfig & config);   // m/s  (prismatic)
float getAngularVelocity(TalonSRX * motor, const MotorConfig & config);  // rad/s (revolute)

// Setters
void  setDutyCycle(TalonSRX * motor, double dutyCycle, int ms);
float setVelocityFromLinearVelocity(TalonSRX * motor, double vel_m_per_s, int ms, const MotorConfig & config);
float setVelocityFromAngularVelocity(TalonSRX * motor, double vel_rad_per_s, int ms, const MotorConfig & config);
float setPositionFromDisplacement(TalonSRX * motor, double displacement_m, int ms, const MotorConfig & config);
float setPositionFromJointCommand(TalonSRX * motor, double position_rad, int ms, const MotorConfig & config);
void  stopMotor(TalonSRX * motor, int ms);

#endif
