/*
 *  LevelWind.h
 *  Implementation of a class to control the level wind
 *  Author: Alex St. Clair
 *  January 2018
 *
 *  This file defines an Arduino library (C++ class) that controls
 *  the level wind. It inherits from the Technosoft class, which
 *  implements communication with the Technosoft motor controllers.
 */

#include "LevelWind.h"

LevelWind::LevelWind(uint8_t expeditor_axis) : Technosoft(LEVEL_WIND_AXIS, expeditor_axis) { }

bool LevelWind::StopProfile() {
	return CallFunction(STOP_PROFILE_LW);
}

#ifdef INST_RACHUTS
// Calls a TML function on the controller. On failure, last_error names it.
bool LevelWind::CallFunctionChecked(uint16_t address, const char * function_name) {
	last_error[0] = '\0';

	if (!CallFunction(address)) {
		snprintf(last_error, sizeof(last_error), "controller rejected %s call", function_name);
		return false;
	}
	return true;
}
#endif

bool LevelWind::SetCenter() {
#ifdef INST_RACHUTS
	return CallFunctionChecked(SET_CENTER_LW, "set center");
#else
	return CallFunction(SET_CENTER_LW);
#endif
}

bool LevelWind::Home() {
#ifdef INST_RACHUTS
	return CallFunctionChecked(HOME_LW, "home");
#else
	return CallFunction(HOME_LW);
#endif
}

bool LevelWind::UpdatePosition() {
	int32_t new_pos = ReadAbsolutePosition();
	if (new_pos != (int32_t) 0xFFFFFFFF) {
		absolute_position = (float) new_pos / LW_STEPS_PER_MM;
		return true;
	}
	return false;
}

bool LevelWind::WindOut(float reel_speed) {
#ifdef INST_RACHUTS
	if (!SetSlewRate((float)(reel_speed*LW_SPEED_CONV))) {
		snprintf(last_error, sizeof(last_error), "controller rejected speed");
		return false;
	}
	delay(50);
	return CallFunctionChecked(WIND_OUT_LW, "wind out");
#else
	if (!SetSlewRate((float)(reel_speed*LW_SPEED_CONV))) { return false; }
	delay(50);
	return CallFunction(WIND_OUT_LW);
#endif
}

bool LevelWind::WindIn(float reel_speed) {
#ifdef INST_RACHUTS
	if (!SetSlewRate((float)(reel_speed*LW_SPEED_CONV))) {
		snprintf(last_error, sizeof(last_error), "controller rejected speed");
		return false;
	}
	delay(50);
	return CallFunctionChecked(WIND_IN_LW, "wind in");
#else
	if (!SetSlewRate((float)(reel_speed*LW_SPEED_CONV))) { return false; }
	delay(50);
	return CallFunction(WIND_IN_LW);
#endif
}
