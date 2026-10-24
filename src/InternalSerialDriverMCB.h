/*
 *  InternalSerialDriverMCB.cpp
 *  File defining the serial driver for communicating with the
 *	main payload computer (DIB) from the MCB
 *  Author: Alex St. Clair
 *  February 2018
 */

#ifndef INTERNALSERIALDRIVERMCB_H_
#define INTERNALSERIALDRIVERMCB_H_

#include "StorageManagerMCB.h"
#include "ActionsMCB.h"
#include "HardwareMCB.h"
#include "MCBComm.h"
#include "SafeBuffer.h"
#include <StdInt.h>

#ifdef INST_RACHUTS
// Buffer size for MCB error TM text: at most 89 characters. StratoCore_RACHuTS
// sends MCB errors to the ground as "MCBString: " + text in a TM StateMess
// field, which the Zephyr interface (STR2-ZEPH-DCI-0-031) limits to 100 chars.
#define MCB_ERROR_BUF_SIZE	90
#endif

struct MCBParameters_t {
	// deploy
	float deploy_length;
	float deploy_velocity;
	float deploy_acceleration;

	// retract
	float retract_length;
	float retract_velocity;
	float retract_acceleration;

	// dock
	float dock_length;
	float dock_velocity;
	float dock_acceleration;

	float temp_limits[6];
	float torque_limits[2];
	float curr_limits[2];
};

class InternalSerialDriverMCB {
public:
	InternalSerialDriverMCB(SafeBuffer * state_q, SafeBuffer * monitor_q);
	~InternalSerialDriverMCB(void) { };

	void RunDriver(void);
	void RunDebugDriver(void);

	void PrintDebugMenu(void);

	MCBComm dibComm;

	MCBParameters_t mcbParameters = {0};

private:
	void HandleASCII(void);

	void PrintDebugCommand(uint8_t cmd, const char * description);

	// Interface objects
	SafeBuffer * state_queue;
	SafeBuffer * monitor_queue;

	StorageManagerMCB storageManager;
};

#endif