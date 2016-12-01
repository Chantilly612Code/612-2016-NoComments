# 1 "./src/Commands/Drive/DriveJoystick.h"
       

#include <Commands/Subsystem.h>
#include "Robot.h"

class DriveJoystick: public Command
{
public:
 DriveJoystick();

 void Initialize();
 void Execute();
 bool IsFinished();
 void End();
 void Interrupted();

private:
 float XDEADZONE = 0.1;
 float JDEADZONE = 0.1;


 float leftPos;
 float rightPos;
};
