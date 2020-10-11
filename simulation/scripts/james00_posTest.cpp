/**
 * @file james00_posTest.cpp
 * @brief Uses iLQR to find optimal controls.  Includes Motor and Winch models.
 * @author James Luo
 */

#include <cstring>
#include <fstream>
#include <iostream>
#include "robot/PositioningFactorGraph.h"
#include "robot/LQRFactorGraph.h"

int main(int argc, char* argv[])
{
	cable_robot::PositioningFactorGraph pos;
	pos.run();
	// cable_robot::LQRFactorGraph lqr;
	// lqr.run();
};