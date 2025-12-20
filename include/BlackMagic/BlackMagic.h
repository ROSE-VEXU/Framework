#ifndef BLACK_MAGIC_H
#define BLACK_MAGIC_H

#define MV(speedPct) speedPct*127.0 
#define VERIFY_SUBCLASS(sub, base, fnName, paramName, superName) static_assert(std::is_base_of<base, sub>::value, fnName " requires parameter " paramName " be a subclass of " superName)
#define VEX_SLEEP_MSEC 10, vex::timeUnits::msec
#define STD_SLEEP_MSEC std::chrono::milliseconds(10)

#include "Config.h"

#include "Direction.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "DriveSpeedProvider.h"
#include "Subsystem.h"
#include "PID.h"
#include "PositionProvider.h"

#include "AutonomousRoutine.h"
#include "AutonomousPipeline.h"
#include "AutonomousSelector.h"
#include "Drivetrain.h"
#include "MotorizedSubsystem.h"
#include "Robot.h"

#endif