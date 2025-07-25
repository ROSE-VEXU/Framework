#ifndef BLACK_MAGIC_H
#define BLACK_MAGIC_H

#define VEX_SLEEP_MSEC 10, vex::timeUnits::msec
#define STD_SLEEP_MSEC std::chrono::milliseconds(10)

#include "AutonomousRoutine.h"
#include "Direction.h"
#include "DriveControllerMovement.h"
#include "DriveMode.h"
#include "DriveSpeedProvider.h"
#include "Subsystem.h"
#include "PID.h"
#include "PositionProvider.h"

#include "AutonomousPipeline.h"
#include "AutonomousSelector.h"
#include "Drivetrain.h"
#include "ControlledSubsystem.h"
#include "Robot.h"

#endif