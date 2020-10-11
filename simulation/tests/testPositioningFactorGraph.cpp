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