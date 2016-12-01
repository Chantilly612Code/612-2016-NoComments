# 1 "./src/Subsystems/ShooterWheels.cpp"
#include <Commands/Shooter/ShooterManualControl.h>
#include "ShooterWheels.h"
#include "RobotMap.h"

ShooterWheels::ShooterWheels() :
  Subsystem("ShooterWheels")
{
 CANTalonLeft = RobotMap::flywheelMotorL;
 CANTalonRight = RobotMap::flywheelMotorR;
 shootertable = new NetworkTables();

 this->hallCounterLeft.reset(new PIDEdgeCounter(RobotMap::flywheelHallL));
 this->hallCounterLeft->Reset();
 this->wheelControllerLeft.reset(new PIDController(this->kP, this->kI, this->kD, this->hallCounterLeft.get(), this->CANTalonLeft.get()));
 this->wheelControllerLeft->SetTolerance(this->kTol);
 this->CANTalonLeft->SetControlMode(CANSpeedController::kPercentVbus);
 this->CANTalonRight->SetInverted(true);
 this->hallCounterRight.reset(new PIDEdgeCounter(RobotMap::flywheelHallR));
 this->wheelControllerRight.reset(new PIDController(this->kP, this->kI, this->kD, this->hallCounterLeft.get(), this->CANTalonRight.get()));
 this->wheelControllerRight->SetTolerance(this->kTol);
    this->CANTalonRight->SetControlMode(CANSpeedController::kPercentVbus);

}

void ShooterWheels::InitDefaultCommand()
{
 SetDefaultCommand(new ShooterManualControl());
}

void ShooterWheels::SetWheelSpeed(float speed)
{
# 51 "./src/Subsystems/ShooterWheels.cpp"
}

float ShooterWheels::GetLeftWheelSpeed()
{
    return this->hallCounterLeft->Get();
}

float ShooterWheels::GetRightWheelSpeed()
{
    return this->hallCounterRight->Get();
}

bool ShooterWheels::UpToSpeed()
{
 shootertable->AddValue(this->hallCounterLeft->Get());
    return this->wheelControllerLeft->OnTarget() && this->wheelControllerRight->OnTarget();
}

void ShooterWheels::Disable()
{
 if(enabled)
 {
  this->wheelControllerLeft->Disable();
  this->wheelControllerRight->Disable();
  this->CANTalonLeft->Disable();
  this->CANTalonRight->Disable();
  this->CANTalonLeft->SetControlMode(CANSpeedController::kVoltage);
  this->CANTalonRight->SetControlMode(CANSpeedController::kVoltage);
  this->CANTalonLeft->Set(0.0f);
  this->CANTalonRight->Set(0.0f);
 }
 enabled = false;
}

void ShooterWheels::Enable()
{
 manualstarted = false;
 if(!enabled)
 {
  this->CANTalonLeft->Enable();
  this->CANTalonRight->Enable();
  this->CANTalonLeft->SetControlMode(CANSpeedController::kPercentVbus);
  this->CANTalonRight->SetControlMode(CANSpeedController::kPercentVbus);
  this->wheelControllerLeft->SetOutputRange(-1.00f, 1.00f);
  this->wheelControllerRight->SetOutputRange(-1.00f, 1.00f);
  this->wheelControllerLeft->Enable();
  this->wheelControllerRight->Enable();
 }
 enabled = true;
}

void ShooterWheels::StartManual()
{
 manualstarted = true;
 this->CANTalonLeft->Enable();
 this->CANTalonRight->Enable();
 this->CANTalonLeft->SetControlMode(CANSpeedController::kPercentVbus);
 this->CANTalonRight->SetControlMode(CANSpeedController::kPercentVbus);
}

void ShooterWheels::ManualSet(float speed)
{
 if(!manualstarted)
 {
  StartManual();
  printf("Starting Manual mode in set");
 }
 this->CANTalonLeft->Set(speed);
 this->CANTalonRight->Set(speed);
}
