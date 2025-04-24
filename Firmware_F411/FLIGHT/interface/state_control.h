#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizer_types.h"

//attitude_pid.h

#include <stdbool.h>
#include "commander.h"
#include "atkp.h"
#include "pid.h"

//position_pid.h

// #include "stabilizer_types.h"
// #include "atkp.h"
// #include "pid.h"

//#define ENABLE_PID_TUNING	/* 使能PID调节 yaw值不更新 */

void stateControlInit(void);
void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick);

//attitude_pid.h

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;

extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

void attitudeControlInit(float rateDt, float angleDt);
void attitudeDataprocess(atkp_t* anlPacket);

void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output);
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);
void attitudeControllerResetRollAttitudePID(void);
void attitudeControllerResetPitchAttitudePID(void);
void attitudeResetAllPID(void);
//void attitudePIDwriteToConfigParam(void);

//position_pid.h

extern PidObject pidVX;
extern PidObject pidVY;
extern PidObject pidVZ;

extern PidObject pidX;
extern PidObject pidY;
extern PidObject pidZ;

void positionControlInit(float ratePidDt, float posPidDt);
void positionDataprocess(atkp_t* anlPacket);
void positionResetAllPID(void);
void positionController(float* thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state, float dt);
float getAltholdThrust(void);

#endif /*__STATE_CONTROL_H */

