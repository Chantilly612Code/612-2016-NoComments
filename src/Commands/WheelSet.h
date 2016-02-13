#ifndef WheelSet_H
#define WheelSet_H

#include "Commands/Command.h"
#include "WPILib.h"

class WheelSet: public Command
{
private:
	float speed;
public:
	WheelSet(float speed);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
