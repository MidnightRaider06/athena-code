#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <cmath>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#define ENCODER_RESOLUTION 64.0 // 14-bit position resolution (Absolute RS485 Encoder)
#define GEAR_RATIO 102.08
#define MAX_RPM 500
#define QUADRATURE_ENCODING 4
#define MEASURED_CPR 5500 // idk why but this is what seems to be the correct cpr (dont use)

#define DISTANCE_PER_REV 0.0015 // in m/rev, 1.5 mm per turn
#define LENGTH_RANGE 6.0 // in inches (because I hate you)

/* Set up motor config */
void initMotor(TalonSRX *motor) {
	std::system("cansend can0 123#00000000");

	motor->ConfigFactoryDefault(100);
	motor->SetInverted(true);

	motor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 100);
	motor->SetSensorPhase(true);

	motor->Config_kP(0, 0.25, 100);
	motor->Config_kD(0, 0.0, 100);
	motor->Config_kF(0, 0.0, 100);
}

// double convertDegreestoTalonSRXUnits(float degrees) {
// 	float outputRotationPerDegree = (1.0 / 360.0);
// 	float motorRotationsPerOutputRotation = (GEAR_RATIO / 1.0); // gearbox
// 	float countsPerMotorRotation = (ENCODER_RESOLUTION / 1.0); // cpr of the encoder
// 	return degrees * outputRotationPerDegree * motorRotationsPerOutputRotation * countsPerMotorRotation;
// }

// double convertTalonSRXUnitsToDegrees(float nativeSensorUnits) {
// 	float motorRotationPerCount = (1.0 / ENCODER_RESOLUTION); // cpr of the encoder
// 	float outputRotationPerMotorRotation = (1.0 / GEAR_RATIO); // gearbox
// 	float degreesPerOutputRotation = (360.0 / 1.0);
// 	return nativeSensorUnits * motorRotationPerCount * outputRotationPerMotorRotation * degreesPerOutputRotation;
// }

double convertRevtoTalonUnits(float rev) {
	// float counts_per_revolution = ENCODER_RESOLUTION*GEAR_RATIO*QUADRATURE_ENCODING;
	float counts_per_revolution = ENCODER_RESOLUTION*GEAR_RATIO;
	return rev * counts_per_revolution;
	// return rev * MEASURED_CPR;
}

double convertTalonUnitstoRev(float counts) {
	// float counts_per_revolution = ENCODER_RESOLUTION*GEAR_RATIO*QUADRATURE_ENCODING;
	float counts_per_revolution = ENCODER_RESOLUTION*GEAR_RATIO;
	return counts / counts_per_revolution;
	// return counts / MEASURED_CPR;
}

double convertRevToDistance(float rev) {
	return rev * DISTANCE_PER_REV;
}

double convertDistanceToRev(float distance) {
	return distance / DISTANCE_PER_REV;
}

// void setDegrees(TalonSRX *motor, double degrees) {
// 	motor.Set(ControlMode::Position, convertRevtoTalonUnits(degrees));
// }

float getPositionTalonUnits(TalonSRX *motor) {
	return motor->GetSelectedSensorPosition(0);
}

float getPositionRevolutions(TalonSRX *motor) {
	// return motor.GetSelectedSensorPosition(0);
	float revs = convertTalonUnitstoRev(motor->GetSelectedSensorPosition(0)); // measures in Talon Units -> revolutions
	return revs;
}

float getPositionDistance(TalonSRX *motor) {
	// return motor.GetSelectedSensorPosition(0);
	float dist = getPositionRevolutions(motor) * DISTANCE_PER_REV;
	return dist;
}

float getVelocityRPM(TalonSRX *motor) {
	float hundred_ms_to_minute = motor->GetSelectedSensorVelocity(0)*600; // measures Talon units per 100ms -> units per minute 
	return convertTalonUnitstoRev(hundred_ms_to_minute); // units per minute -> rev per minute (RPM)
}

float getClawVelocity(TalonSRX *motor) {
	float hundred_ms_to_seconds = motor->GetSelectedSensorVelocity(0)*10; // measures Talon units per 100ms -> units per second
	float units_to_rev = convertTalonUnitstoRev(hundred_ms_to_seconds); // units per second -> rev/sec
	float m_per_sec = units_to_rev * DISTANCE_PER_REV; // rev/sec to m/s
	return m_per_sec;
}

void setDutyCycle(TalonSRX *motor, double dutyCycle, int ms) {
	motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, dutyCycle);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds
}

float setVelocityFromLinearVelocity(TalonSRX *motor, double claw_vel, int ms) {
	// float desired_rpm = joy_input * MAX_RPM;
	// float desired_rev_per_100ms = desired_rpm / 600; // 600 100ms to 1 minute

	//TO DO: why does the 5 fix this. I dont have a clue rn
	float desired_talon_units_per_100ms = convertRevtoTalonUnits(convertDistanceToRev((claw_vel*5)/10)); // m/s -> m/100ms -> rev/100ms -> talon/100ms

	motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, desired_talon_units_per_100ms);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds

	return desired_talon_units_per_100ms;
}

float setVelocityFromAngularVelocity(TalonSRX *motor, double joint_velocity, int ms) {
	float desired_talon_units_per_100ms = convertRevtoTalonUnits(joint_velocity / (2*M_PI*10)); // rad/s -> rad/100ms -> rev/100ms -> talon/100ms

	motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, desired_talon_units_per_100ms);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds

	return desired_talon_units_per_100ms;
}

float setPositionFromDisplacement(TalonSRX *motor, double claw_displacement, int ms) {
	// float length_range = LENGTH_RANGE * 2.54; // in cm
	// float desired_pos = (joy_input * length_range) / DISTANCE_PER_REV; // in rev
	// float desired_talon_units = convertRevtoTalonUnits(desired_pos);

	float desired_talon_units = convertRevtoTalonUnits(convertDistanceToRev(claw_displacement)); // m -> rev -> talon units
	motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, desired_talon_units);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds

	return desired_talon_units;
}

float setPositionFromJointCommand(TalonSRX *motor, double joint_position, int ms) {
	float desired_talon_units = convertRevtoTalonUnits(joint_position / (2*M_PI)); // rad -> rev -> talon units
	motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, desired_talon_units);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds

	return desired_talon_units;
}

void stopMotor(TalonSRX *motor, int ms){
	motor->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0.0);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds

}

// float runMotor(TalonSRX *motor, double speed, int ms) {
// 	float velocity = setVelocityFromLinearVelocity(motor, speed); // motor receives talon units/100ms, function returns rev/s
// 	// setDutyCycle(motor, speed); // set percent output and direction
// 	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds
// 	return velocity;
// }

// float setMotorPosition(TalonSRX *motor, double joy_input, int ms) {
// 	float position = setPositionFromDisplacement(motor, joy_input); // in Talon Units
// 	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms); // activate motor for (ms) milliseconds
// 	return position;
// }

// void closeClaw(TalonSRX *motor) {
// 	while (abs(motor.GetStatorCurrent()) < 0.1) {
// 		runMotor(motor, -1, 10);
// 		sleep(0.01);
// 	}
// }