# 1 "./src/Commands/Autonomous/Sequences/HorizontalAlign.h"
#ifndef AlignToTarget_H
#define AlignToTarget_H 

#include "Robot.h"
#include "WPILib.h"
#include "VisionTarget.h"
#include <cmath>

class HorizontalAlign: public PIDCommand
{
public:
 HorizontalAlign(float timeout = 0);
 void Initialize();
 void Execute();
 bool IsFinished();
 void End();
 void Interrupted();

 double ReturnPIDInput();
 void UsePIDOutput(double output);

private:
 bool hasTarget = false;
 bool PIDUserDisabled = true;
 int onTargetCounter = 0;

 void FindTarget();
 const double TARGET_ASPECT = 1.66/1.00;
 const int SCREEN_CENTER_X = 320;


 const double ROT_SPEED_CAP = 0.8;
 const double ROT_SPEED_MIN = 0.4;
};

#endif
