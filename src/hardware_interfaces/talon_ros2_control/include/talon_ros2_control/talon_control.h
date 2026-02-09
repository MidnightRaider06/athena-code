#ifndef TALON_CONTROL_H
#define TALON_CONTROL_H

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

// TalonSRX motor(8);

void initMotor(TalonSRX *);

// Conversions
double convertRevtoTalonUnits(float); // Converts Revolutions to Talon Units
double convertTalonUnitstoRev(float); // Converts Talon units to Revolutions
double convertRevToDistance(float);  // Converts revolutions of motor to meters
double convertDistanceToRev(float); // Converts meters to revolutions of motor

// Getters
float getPositionTalonUnits(TalonSRX *); // Retrieves Position in Talon Encoder Units
float getPositionRevolutions(TalonSRX *); // Retrieves Position in Revolutions from start
float getPositionDistance(TalonSRX *); // Retrieves Position in meters from start
float getVelocityRPM(TalonSRX *); // Retrieves Velocity for the motor in RPM
float getClawVelocity(TalonSRX *); // Retrieves Velocity of claw in m/s
// void setDegrees(TalonSRX *, double);

// Setting States
void setDutyCycle(TalonSRX *, double, int); // Sets duty cycle, aka percent of max speed basically
float setVelocityFromLinearVelocity(TalonSRX *, double, int); // Sets velocity (units per 100ms) based on input velocity (mm/s), returns this as well
float setVelocityFromAngularVelocity(TalonSRX *, double, int); // Sets velocity (units per 100ms) based on input velocity (radians/s), returns this as well
float setPositionFromDisplacement(TalonSRX *, double, int); // Sets position (Talon units) based on input displacement (cm), returns this as well
float setPositionFromJointCommand(TalonSRX *, double, int); // Sets position (Talon units) based on input joint command (radians), returns this as well
void stopMotor(TalonSRX *, int); // Stops the motor

#endif