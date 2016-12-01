# 1 "./src/Commands/Drive/SetGear.h"
       
#include "WPILib.h"
#include "Robot.h"

class SetGear: public Command
{
public:
 SetGear(float gear);
 void Initialize();
 void Execute();
 bool IsFinished();
 void End();
 void Interrupted();
private:
 float gear;
};
