/*
 *  Reel.h
 *  Implementation of a class to control the reel
 *  Author: Alex St. Clair
 *  January 2018
 *
 *  This file defines an Arduino library (C++ class) that controls
 *  the reel. It inherits from the Technosoft class, which implements
 *  communication with the Technosoft motor controllers.
 */

#include "Reel.h"

Reel::Reel(uint8_t expeditor_axis) : Technosoft(REEL_AXIS, expeditor_axis) {
	// keep emergency stop off to start
	pinMode(MC1_IN4_PIN, OUTPUT); // limit switch negative (emergency stop)
	digitalWrite(MC1_IN4_PIN, HIGH);
	absolute_position = 0;
	speed = 0.0f;
}

bool Reel::SetPosition(float new_pos) {
	int32_t cmd_pos = (int32_t) (new_pos * REEL_UNITS_PER_REV);
	if (SetAbsolutePosition(cmd_pos)) {
		absolute_position = cmd_pos;
		UpdatePosition(); // will log to file
		return true;
	}
	// TODO: error checking
	return false;
}

bool Reel::SetPosition(int32_t new_pos) {
	if (SetAbsolutePosition(new_pos)) {
		absolute_position = new_pos;
		UpdatePosition(); // will log to file
		return true;
	}
	// TODO: error checking
	return false;
}

bool Reel::SetPositionDeferred(float new_pos) {
	return SetPositionDeferred((int32_t) (new_pos * REEL_UNITS_PER_REV));
}

// Records a known position without touching the controller, so the motor
// doesn't briefly energize/release the brake (SetAbsolutePosition does this
// on our controllers) while the RPU may be docked against the gondola.
// The actual controller write is deferred until the next real motion
// command, via CommitDeferredPosition() in ReelIn()/ReelOut().
bool Reel::SetPositionDeferred(int32_t new_pos) {
	absolute_position = new_pos;
	pending_position = new_pos;
	position_write_pending = true;
	return storageManager.WriteSD_int32(POS_LOG_FILE, new_pos, false, 0);
}

bool Reel::CommitDeferredPosition() {
	if (!position_write_pending) return true;

	if (!SetAbsolutePosition(pending_position)) {
		// leave pending so the next motion command can retry
		return false;
	}

	position_write_pending = false;
	return true;
}

bool Reel::UpdatePosition() {
	// don't let a stale read of the controller's not-yet-updated register
	// clobber a position that's cached locally awaiting CommitDeferredPosition()
	if (position_write_pending) return true;

	int32_t new_pos = ReadAbsolutePosition();
	if (new_pos != (int32_t) 0xFFFFFFFF) {
		absolute_position = new_pos;
		storageManager.WriteSD_int32(POS_LOG_FILE, new_pos, false, 0);
		return true;
	}
	return false;
}

bool Reel::UpdateSpeed() {
	speed = ReadActualSpeed();
	return 0.0f == speed;
}

// On RACHuTS, restoring the position after a (re)boot is deferred: writing to
// the controller via SetAbsolutePosition briefly energizes the motor/releases
// the brake on our controllers. That's commanded here just from powering the
// reel controller on (e.g. for HomeLW/CenterLW), with no reel motion
// necessarily following, so it carries the same dock-slack risk as zeroing.
// The value is cached now and flushed to the controller by
// CommitDeferredPosition() the next time the reel actually turns.
// RATS and FLOATS keep the original immediate write.
void Reel::SetToStoredPosition() {
	if (!storageManager.FileExists(POS_LOG_FILE)) {
#ifdef INST_RACHUTS
		SetPositionDeferred((int32_t) absolute_position);
#else
		SetPosition((int32_t) absolute_position);
#endif
		return;
	}

	int32_t read_pos = 0;
	if (storageManager.ReadSD_int32(POS_LOG_FILE, &read_pos, 0)) {
#ifdef INST_RACHUTS
		SetPositionDeferred(read_pos);
#else
		SetPosition(read_pos);
#endif
	} else {
#ifdef INST_RACHUTS
		SetPositionDeferred((int32_t) absolute_position);
#else
		SetPosition((int32_t) absolute_position);
#endif
	}
}

bool Reel::StopProfile() {
	return CallFunction(STOP_PROFILE_R);
}

#ifdef INST_RACHUTS
// Validates motion parameters against the instrument limits in Reel.h. On
// failure, last_error describes which parameter was out of range so callers
// can include it in the error TM.
bool Reel::CheckMotionParams(float num_revolutions, float speed, float acc) {
	last_error[0] = '\0';

	if (num_revolutions > MAX_REVOLUTIONS || num_revolutions <= 0.0) {
		snprintf(last_error, sizeof(last_error), "revs %.4g not in (0, %.1f]", num_revolutions, MAX_REVOLUTIONS);
		return false;
	}
	if (speed > MAX_SPEED || speed <= 0.0) {
		snprintf(last_error, sizeof(last_error), "speed %.4g not in (0, %.1f] rpm", speed, (float) MAX_SPEED);
		return false;
	}
	if (acc > MAX_ACC || acc <= 0.0) {
		snprintf(last_error, sizeof(last_error), "acc %.4g not in (0, %.2f] rot/s^2", acc, (float) MAX_ACC);
		return false;
	}
	return true;
}

// Describes a Technosoft Read16BitRAM/Read32BitRAM return code
const char * TechnosoftReadErrorName(int code)
{
	switch (code) {
	case 1:   return "readback failed once"; // the diagnostic re-read worked
	case -10: return "serial write failed";
	case -11: return "no reply";
	case -12: return "bad ack";
	case -13: return "no data after ack";
	case -14: return "port not initialized";
	case -21: return "bad checksum";
	case -4:  return "wrong axis reply";
	case -6:  return "wrong register reply";
	default:  return "bad reply";
	}
}

int Reel::ProbeStatus(void) {
	uint16_t status = 0;
	return Read16BitRAM(STATUS_LO_R, &status);
}

// Writes a 32-bit controller register, verified by read-back (as
// SetCommandPosition, SetSlewRate and SetAcceleration do). On failure,
// last_error says which register and why.
bool Reel::WriteRegister(uint16_t address, uint32_t value, const char * name) {
	int ret = Write32BitRAM(address, value);
	if (1 == ret) return true;

	const char * reason = "write failed";
	if (-4 == ret) {
		reason = "readback mismatch"; // controller answered, but the write didn't take
	} else if (-3 == ret) {
		// Write32BitRAM doesn't say why the read-back failed, so read once more to find out
		uint32_t readback = 0;
		reason = TechnosoftReadErrorName(Read32BitRAM(address, &readback));
	}
	snprintf(last_error, sizeof(last_error), "controller rejected %s (%s)", name, reason);
	return false;
}

// Sends an already-converted motion profile to the controller and starts it.
// On failure, last_error names the step that failed.
bool Reel::CommandMotion(uint32_t num_units, uint32_t fixed_speed, uint32_t fixed_acc) {
	if (!WriteRegister(CPOS_P, num_units, "position")) return false;
	if (!WriteRegister(CSPD_P, fixed_speed, "speed")) return false;
	if (!WriteRegister(CACC_P, fixed_acc, "acc")) return false;
	if (!CallFunction(REEL_VARIABLE_R)) {
		snprintf(last_error, sizeof(last_error), "controller rejected motion call");
		return false;
	}
	return true;
}
#endif

bool Reel::ReelIn(float num_revolutions, float speed, float acc) {
	uint32_t num_units = 0;
	uint32_t fixed_speed = 0;
	uint32_t fixed_acc = 0;

#ifdef INST_RACHUTS
	if (!CheckMotionParams(num_revolutions, speed, acc)) { return false; }
#else
	if (num_revolutions > MAX_REVOLUTIONS || num_revolutions <= 0.0) { 
		Serial.println("Revolutions Wrong");
		return false; }
	if (speed > MAX_SPEED || speed <= 0.0) { 
		Serial.println("Speed Wrong");
		return false; }
	if (acc > MAX_ACC || acc <= 0.0) {
		Serial.println("Acc Wrong");
		return false; }
#endif

	// the reel is about to actually turn, so it's now safe to flush any
	// position that was cached (rather than written) while stationary
	if (!CommitDeferredPosition()) {
		Serial.println("Warning: unable to commit deferred reel position");
	}

	// implicit cast to uint32 for serialization
	num_units = num_revolutions * REEL_UNITS_PER_REV;

	// Technosoft unit conversions
	speed = speed * SPEED_CONVERSION;
	acc = acc * ACC_CONVERSION;

	// convert floats to fixed points
	fixed_speed = Float_To_Fixed(speed);
	fixed_acc = Float_To_Fixed(acc);

#ifdef INST_RACHUTS
	return CommandMotion(num_units, fixed_speed, fixed_acc);
#else
	if (!SetCommandPosition(num_units)) { 
		Serial.println("SetCommandPosition Wrong");
		return false; }
	if (!SetSlewRate(fixed_speed)) { 
		Serial.println("SetSlewRate Wrong");
		return false; }
	if (!SetAcceleration(fixed_acc)) { 
		Serial.println("SetAcceleration Wrong");
		return false; }

	return CallFunction(REEL_VARIABLE_R);
#endif
}

bool Reel::ReelOut(float num_revolutions, float speed, float acc) {
	uint32_t num_units = 0;
	uint32_t fixed_speed = 0;
	uint32_t fixed_acc = 0;

#ifdef INST_RACHUTS
	if (!CheckMotionParams(num_revolutions, speed, acc)) { return false; }
#else
	if (num_revolutions > MAX_REVOLUTIONS || num_revolutions <= 0.0) { return false; }
	if (speed > MAX_SPEED || speed <= 0.0) { return false; }
	if (acc > MAX_ACC || acc <= 0.0) { return false; }
#endif

	// the reel is about to actually turn, so it's now safe to flush any
	// position that was cached (rather than written) while stationary
	if (!CommitDeferredPosition()) {
		Serial.println("Warning: unable to commit deferred reel position");
	}

	// cast as int32 first to get sign before implicit cast to uint32 for serialization
	num_units = (int32_t) (num_revolutions * REEL_UNITS_PER_REV * -1);

	// Technosoft unit conversions
	speed = speed * SPEED_CONVERSION;
	acc = acc * ACC_CONVERSION;

	// convert floats to fixed points
	fixed_speed = Float_To_Fixed(speed);
	fixed_acc = Float_To_Fixed(acc);

#ifdef INST_RACHUTS
	return CommandMotion(num_units, fixed_speed, fixed_acc);
#else
	if (!SetCommandPosition(num_units)) { return false; }
	if (!SetSlewRate(fixed_speed)) { return false; }
	if (!SetAcceleration(fixed_acc)) { return false; }

	return CallFunction(REEL_VARIABLE_R);
#endif
}

bool Reel::CamSetup() {
	return CallFunction(CAM_SETUP_R);
}

bool Reel::CamStop() {
	return CallFunction(CAM_STOP_R);
}

bool Reel::BrakeOn() {
	return CallFunction(BRAKE_ON_R);
}

bool Reel::BrakeOff() {
	return CallFunction(BRAKE_OFF_R);
}

void Reel::EmergencyStop() {
	digitalWrite(MC1_IN4_PIN, LOW);
}

void Reel::RemoveEmergencyStop() {
	digitalWrite(MC1_IN4_PIN, HIGH);
}