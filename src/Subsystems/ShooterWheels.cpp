#include "ShooterWheels.h"

ShooterWheels::ShooterWheels() :
		Subsystem("ShooterWheels")
{
	CANTalonLeft = RobotMap::leftFlywheel;
	CANTalonRight = RobotMap::rightFlywheel;
	this->hallCounterLeft.reset(new PIDEdgeCounter(RobotMap::leftFlywheelHall));
	this->wheelControllerLeft.reset(new PIDController(this->kP, this->kI, this->kD, this->hallCounterLeft.get(), this->CANTalonLeft.get()));
	this->wheelControllerLeft->SetTolerance(this->kTol);
	this->hallCounterRight.reset(new PIDEdgeCounter(RobotMap::rightFlywheelHall));
	this->wheelControllerRight.reset(new PIDController(this->kP, this->kI, this->kD, this->hallCounterLeft.get(), this->CANTalonRight.get()));
	this->wheelControllerRight->SetTolerance(this->kTol);
}

void ShooterWheels::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	//SetDefaultCommand(new MySpecialCommand());

}
void ShooterWheels::setWheelSpeed(float speed)
{
    this->wheelControllerLeft->Enable();
    this->wheelControllerLeft->SetOutputRange(-1.00f, 1.00f);
    this->wheelControllerLeft->SetSetpoint(speed);
    this->wheelControllerRight->Enable();
    this->wheelControllerRight->SetOutputRange(-1.00, 1.00);
    this->wheelControllerRight->SetSetpoint(speed);
}

float ShooterWheels::getLeftWheelSpeed()
{
	return this->wheelControllerLeft->Get();
}

float ShooterWheels::getRightWheelSpeed()
{
    return this->wheelControllerRight->Get();
}

bool ShooterWheels::upToSpeed()
{
    return this->wheelControllerLeft->OnTarget() && this->wheelControllerRight->OnTarget();
}
