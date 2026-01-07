#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "BlackMagic/BlackMagic.h"

#include "extern.h"
#include "misc-custom-modules/ArcadeDriveControl.h"
#include "subsystems/accessories.h"
#include "subsystems/intake.h"
#include "subsystems/lever.h"
#include "auto/LimitSwitchAutoSelector.h"
#include "auto/DriveToPose.h"
#include "auto/SimpleDriveToPoint.h"
#include "auto/DriveToPoint.h"
#include "auto/RotationalOdometry.h"
#include "auto/RotationalOdometryTester.h"
#include "auto/autos.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)