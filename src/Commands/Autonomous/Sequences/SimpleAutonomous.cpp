# 1 "./src/Commands/Autonomous/Sequences/SimpleAutonomous.cpp"
#include "SimpleAutonomous.h"

#include "Robot.h"
#include "RobotMap.h"

SimpleAutonomous::SimpleAutonomous(float time, float speed)
{
 Requires(Robot::drivetrain.get());

 SetTimeout(time);
 this->speed = speed;
 original_speed = speed;

}

void SimpleAutonomous::Initialize()
{
 Robot::drivetrain.get()->SetArcadeDrive(speed, 0);
}

void SimpleAutonomous::Execute()
{

 if(!(abs(RobotMap::NavX->GetPitch()) < THRESHOLD) ||
   !(abs(RobotMap::NavX->GetRoll()) < THRESHOLD))





 {
  std::printf("Info: Incrementing speed\n");

  speed *= TIMES_INCREMENT;
 }


 if(!(abs(RobotMap::NavX->GetYaw()) < MAX_YAW_ERROR))

 {
  if(RobotMap::NavX->GetYaw() > 0)
  {





   rotation -= ADD_INCREMENT;

  }
  else if(RobotMap::NavX->GetYaw() < 0)
  {




   rotation += ADD_INCREMENT;
  }
 }
 else

 {
  rotation = 0;
 }


 if(abs(RobotMap::NavX->GetYaw()) < MAX_YAW_ERROR &&
   abs(RobotMap::NavX->GetPitch()) < THRESHOLD &&
   abs(RobotMap::NavX->GetRoll()) < THRESHOLD)





 {
  std::printf("Info: Speed reset\n");

  speed = original_speed;
  rotation = 0;
 }


 if(speed > 1)
  std::printf("Warning: Maxing out motors!\n");

 std::printf("Info: Motor Speed: %f, Rotation Speed: %f\n", speed, rotation);
 std::printf("Info: Yaw: %f\n", RobotMap::NavX.get()->GetYaw());
 Robot::drivetrain->SetArcadeDrive(speed, rotation);
}

bool SimpleAutonomous::IsFinished()
{
 return IsTimedOut();
}

void SimpleAutonomous::End()
{
 printf("Info: End auto driving.\n");
 Robot::drivetrain->SetTankDrive(0.0f, 0.0f);
}

void SimpleAutonomous::Interrupted()
{
 printf("Warning: Autonomous driving interrupted\n");
 Robot::drivetrain->SetTankDrive(0.0f, 0.0f);
}
