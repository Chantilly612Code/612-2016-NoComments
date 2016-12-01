# 1 "./src/Commands/Autonomous/Sequences/VerticalAlign.cpp"
#include "VerticalAlign.h"

VerticalAlign::VerticalAlign(float timeout)
{
 Requires(Robot::shooterrotation.get());
 printf("va const\n");

 if(timeout != 0)
  SetTimeout(timeout);
}

void VerticalAlign::Initialize()
{
 tID = -1;
 AlignShooter();
 printf("va init\n");
}

void VerticalAlign::Execute()
{
 if (Robot::vision->GetTrackingID() != tID)
  AlignShooter();
 printf("va exe\n");
}

bool VerticalAlign::IsFinished()
{

 char* output = "";
 if(Robot::shooterrotation->OnTarget())
  output = "onTarget";
 else if (tID == -1)
  output = "tID";
 SmartDashboard::PutString("VerticalAlign", output);
 printf("VerticalAlign %s\n", output);

 return Robot::shooterrotation->OnTarget() || tID == -1;
}

void VerticalAlign::End()
{
 SmartDashboard::PutString("VerticalAlign end", "Ended");
 Robot::shooterrotation->PIDEnable(false);
}

void VerticalAlign::Interrupted()
{
 Robot::shooterrotation->PIDEnable(false);
}

void VerticalAlign::AlignShooter()
{
 int nID = Robot::vision->GetTrackingID();

 if (nID != -1)
 {
  std::shared_ptr<VisionTarget> target = Robot::vision->GetTrackedGoal();
  double angle = CalcAngle(target->GetDistance());
  SmartDashboard::PutNumber("VA Angle", angle);
  SmartDashboard::PutNumber("VA Distance", Robot::vision->GetTrackedGoal()->GetDistance());
  Robot::shooterrotation->SetAngle(angle);
 }

 tID = nID;
}

double VerticalAlign::CalcAngle(double dist)
{

 double x = dist;
 SmartDashboard::PutNumber("Distance", x);
 double y = TARGET_HEIGHT_OFF_GROUND;
 double v = BOULDER_LAUNCH_SPEED;
 double g = 9.8;
 double sq = pow(v, 4) - (g * (g * (x * x) + 2 * y * (v * v)));

 sq = sqrt(abs(sq));

 double angleInRadians = atan(((v * v) - sq) / (g * x));
 double angleInDegrees = angleInRadians * (180 / 3.14159265358979323846264338327950288419716939937510);
 SmartDashboard::PutNumber("Calculated Angle", angleInDegrees);
 return angleInDegrees;
}
