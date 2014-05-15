/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// Accelerometer Calibration
///////////////////////////////////////////////////////////////////////////////

void accelCalibrationMXR(void)
{
    float noseUp        = 0.0f;
	float noseDown      = 0.0f;
	float leftWingDown  = 0.0f;
	float rightWingDown = 0.0f;
	float upSideDown    = 0.0f;
    float rightSideUp   = 0.0f;

    float xBias         = 0.0f;
    float yBias         = 0.0f;
    float zBias         = 0.0f;

    int16_t index;

    accelCalibrating = true;

    cliPortPrint("\nMXR9150 Accelerometer Calibration:\n\n");

    ///////////////////////////////////

    cliPortPrint("Place accelerometer right side up\n");
    cliPortPrint("  Send a character when ready to proceed\n\n");

    while (cliPortAvailable() == false);
    cliPortRead();

    cliPortPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        rightSideUp += mxr9150ZAxis();

        xBias += mxr9150XAxis();
        yBias += mxr9150YAxis();

        delayMicroseconds(1000);
    }

    rightSideUp /= 5000.0f;

    cliPortPrint("Place accelerometer up side down\n");
    cliPortPrint("  Send a character when ready to proceed\n\n");

    while (cliPortAvailable() == false);
    cliPortRead();

    cliPortPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        upSideDown += mxr9150ZAxis();

        xBias += mxr9150XAxis();
        yBias += mxr9150YAxis();

        delayMicroseconds(1000);
    }

    upSideDown /= 5000.0f;

    ///////////////////////////////////

    cliPortPrint("Place accelerometer left edge down\n");
    cliPortPrint("  Send a character when ready to proceed\n\n");

    while (cliPortAvailable() == false);
    cliPortRead();

    cliPortPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        leftWingDown += mxr9150YAxis();

        zBias += mxr9150ZAxis();

        delayMicroseconds(1000);
    }

    leftWingDown /= 5000.0f;

    cliPortPrint("Place accelerometer right edge down\n");
    cliPortPrint("  Send a character when ready to proceed\n\n");

    while (cliPortAvailable() == false);
    cliPortRead();

    cliPortPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        rightWingDown += mxr9150YAxis();

        zBias += mxr9150ZAxis();

        delayMicroseconds(1000);
    }

    rightWingDown /= 5000.0f;

    ///////////////////////////////////

    cliPortPrint("Place accelerometer rear edge down\n");
    cliPortPrint("  Send a character when ready to proceed\n\n");

    while (cliPortAvailable() == false);
    cliPortRead();

    cliPortPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        noseUp += mxr9150XAxis();

        zBias += mxr9150ZAxis();

        delayMicroseconds(1000);
    }

    noseUp /= 5000.0f;

    cliPortPrint("Place accelerometer front edge down\n");
    cliPortPrint("  Send a character when ready to proceed\n\n");

    while (cliPortAvailable() == false);
    cliPortRead();

    cliPortPrint("  Gathering Data...\n\n");

    for (index = 0; index < 5000; index++)
    {
        noseDown += mxr9150XAxis();

        zBias += mxr9150ZAxis();

        delayMicroseconds(1000);
    }

    noseDown /= 5000.0f;

    ///////////////////////////////////

    eepromConfig.accelBiasMXR[ZAXIS]        = zBias / 20000.0f;
    eepromConfig.accelScaleFactorMXR[ZAXIS] = (2.0f * 9.8065f) / fabsf(rightSideUp - upSideDown);


    eepromConfig.accelBiasMXR[YAXIS]        = yBias / 10000.0f;
    eepromConfig.accelScaleFactorMXR[YAXIS] = (2.0f * 9.8065f) / fabsf(leftWingDown - rightWingDown);


    eepromConfig.accelBiasMXR[XAXIS]        = xBias / 10000.0f;
    eepromConfig.accelScaleFactorMXR[XAXIS] = (2.0f * 9.8065f) / fabsf(noseUp - noseDown);

    ///////////////////////////////////

	accelCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
