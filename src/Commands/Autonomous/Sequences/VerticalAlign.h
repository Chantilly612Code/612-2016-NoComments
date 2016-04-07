#ifndef VerticalAlign_H
#define VerticalAlign_H

#include "WPILib.h"
#include <Robot.h>

class VerticalAlign: public Command
{
public:
	VerticalAlign();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

	const double TARGET_HEIGHT_OFF_GROUND = 2; //in meters
	const double BOULDER_LAUNCH_SPEED = 12.9; //in meters per second


private:
	int tID = 0;

	double CalcAngle(double dist);
	void AlignShooter();
};

#endif