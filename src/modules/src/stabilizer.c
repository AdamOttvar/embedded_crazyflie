/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#define DEBUG_MODULE "STAB"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"
#include "position_estimator.h"
#include "position_controller.h"
#include "altitude_hold.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define ATTITUDE_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 100hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg

uint16_t actuatorThrust;  // Actuator output for thrust base

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static float reference[4];

static bool isInit;
static bool isInitModeSwitcher;
static bool isInitRefMaker;

QueueHandle_t xQueueRef, xQueueMode;

static uint16_t limitThrust(int32_t value);

static void stabilizerTask(void* param)
{
  uint32_t motorCounter, modeCounter;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {

    	if (xQueueReceive(xQueueMode, &modeCounter, M2T(10))) {
			DEBUG_PRINT("----------------------------\n");
			DEBUG_PRINT("Received: %u \n", (unsigned int)modeCounter);
		}

    	if (xQueueReceive(xQueueRef, &motorCounter, M2T(10))) {
    		DEBUG_PRINT("----------------------------\n");
    		DEBUG_PRINT("Received: %u \n", (unsigned int)motorCounter);
    	}

		if (motorCounter == 1) {
			motorPowerM1 = limitThrust(2000 + 5000*modeCounter);
		}
		else if (motorCounter == 2) {
			motorPowerM2 = limitThrust(2000 + 5000*modeCounter);
		}
		else if (motorCounter == 3) {
			motorPowerM3 = limitThrust(2000 + 5000*modeCounter);
		}
		else if (motorCounter == 4) {
			motorPowerM4 = limitThrust(2000 + 5000*modeCounter);
		}
		else {
			motorPowerM2 = limitThrust(0.0);
			motorPowerM3 = limitThrust(0.0);
			motorPowerM1 = limitThrust(0.0);
			motorPowerM4 = limitThrust(0.0);
		}

		motorsSetRatio(MOTOR_M1, motorPowerM1);
		motorsSetRatio(MOTOR_M2, motorPowerM2);
		motorsSetRatio(MOTOR_M3, motorPowerM3);
		motorsSetRatio(MOTOR_M4, motorPowerM4);

    }
  }
}

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  attitudeControllerInit();
  modeSwitcherInit();
  refMakerInit();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= attitudeControllerTest();

  return pass;
}

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}

void modeSwitcher(void* param)
{
	uint32_t lastWakeTime;
	uint32_t modeCounter = 0;
	systemWaitStart();
	lastWakeTime = xTaskGetTickCount ();

	while(1)
	  {
		*(reference + 0) = 0;
		*(reference + 1) = 0;
		*(reference + 2) = 0;
		*(reference + 3) = 0;
	    vTaskDelayUntil(&lastWakeTime, M2T(10000)); // Wait 9 seconds
	    xQueueSendToBack(xQueueMode, &modeCounter, M2T(10));
	  }

}

void modeSwitcherInit(void)
{
	if(isInitModeSwitcher)
	    return;

	xTaskCreate(modeSwitcher, MODE_SWITCHER_TASK_NAME,
			MODE_SWITCHER_STACKSIZE, NULL, MODE_SWITCHER_TASK_PRI, NULL);

	xQueueMode = xQueueCreate(1,sizeof(uint32_t));

	isInitModeSwitcher = true;
}


void refMaker(void* param)
{
	uint32_t lastWakeTime;
	uint32_t motorCounter = 0;
	systemWaitStart();
	lastWakeTime = xTaskGetTickCount ();

	while(1)
	  {

		if (++motorCounter > 5) {
			motorCounter = 1;
		}

	    vTaskDelayUntil(&lastWakeTime, M2T(2000)); // Wait 1 second
	    xQueueSendToBack(xQueueRef, &motorCounter, M2T(10));
	  }
}

void refMakerInit(void)
{
	if(isInitRefMaker)
	    return;

	xTaskCreate(refMaker, REF_MAKER_TASK_NAME,
			REF_MAKER_STACKSIZE, NULL, REF_MAKER_TASK_PRI, NULL);

	xQueueRef = xQueueCreate(1,sizeof(uint32_t));

	isInitRefMaker = true;
}


LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)

