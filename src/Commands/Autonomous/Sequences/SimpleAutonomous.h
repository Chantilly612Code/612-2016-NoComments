# 1 "./src/Commands/Autonomous/Sequences/SimpleAutonomous.h"
       

#include "Commands/CommandGroup.h"
#include "Robot.h"
#include <Timer.h>

class SimpleAutonomous: public CommandGroup
{
private:
 float speed;
 float original_speed;
 float rotation = 0;

 const int THRESHOLD = 5;
 const int MAX_YAW_ERROR = 3;
 const float TIMES_INCREMENT = 1.005;
 const float ADD_INCREMENT = 0.01;
public:
 SimpleAutonomous(float time, float speed);
 void Initialize();
 void Execute();
 bool IsFinished();
 void End();
 void Interrupted();
};
