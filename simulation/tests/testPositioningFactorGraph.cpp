#include <cstring>
#include <fstream>
#include <iostream>
#include "PositioningFactorGraph.h"
#include "LQRFactorGraph.h"

int main(int argc, char* argv[])
{
	cable_robot::PositioningFactorGraph pos;
	pos.run();
	// cable_robot::LQRFactorGraph lqr;
	// lqr.run();
	return 1;

};