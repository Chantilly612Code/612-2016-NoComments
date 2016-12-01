# 1 "./src/Commands/Shooter/ShooterJoystick.cpp"
#include "ShooterJoystick.h"
#include <CANSpeedController.h>
#include "OI.h"



ShooterJoystick::ShooterJoystick()
{
 Requires(Robot::shooterrotation.get());
}

void ShooterJoystick::Initialize() { }

void ShooterJoystick::Execute()
{
 auto joy = Robot::oi->getGunner();
 bool b = joy->GetRawButton(2),
   x = joy->GetRawButton(3), y = joy->GetRawButton(4);
 bool buttonPushed = b | x | y;

 if(!buttonPushed)
 {
  if(Robot::shooterrotation->PIDEnabled())
   Robot::shooterrotation->PIDEnable(false);


  auto gunner = joy->GetRawAxis(5);
  if(gunner < TOLERANCE && gunner > -TOLERANCE)
   gunner = 0;

  #ifdef TEST_PID
  Robot::shooterrotation->SetAngle(SmartDashboard::GetNumber("Shooter Angle", 0));
  #else

  Robot::shooterrotation->Gun(gunner / 2);
  #endif
 }
 else
 {
  if(!Robot::shooterrotation->PIDEnabled())
   Robot::shooterrotation->PIDEnable(true);
  if(x) Robot::shooterrotation->HomePos();
  if(b) Robot::shooterrotation->IntakePos();
  if(y) Robot::shooterrotation->SetAngle(Robot::shooterrotation->HIGOAL_ANGLE);
 }
}

bool ShooterJoystick::IsFinished()
{
 return false;
}

void ShooterJoystick::End()
{
 Robot::shooterrotation->Stop();
}

void ShooterJoystick::Interrupted()
{
 Robot::shooterrotation->Stop();
}
