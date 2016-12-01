# 1 "./src/Commands/Drive/DriveDistance.cpp"
#include "DriveDistance.h"
#include "Robot.h"

DriveDistance::DriveDistance(float end_distance) : PIDCommand("DriveDistance", 0.2, 0.0, 0.0)
{
 Requires(Robot::drivetrain.get());

 RobotMap::driveEncoderL->Reset();
 this->end_distance = end_distance;

 GetPIDController()->SetContinuous(true);
 GetPIDController()->SetOutputRange(-1.0f, 1.0f);
 GetPIDController()->SetPercentTolerance(0.05);
}

void DriveDistance::Initialize()
{
 GetPIDController()->SetSetpoint(end_distance);
 Robot::drivetrain->SetTankDrive(SPEED, SPEED);
 GetPIDController()->SetInputRange(STARTING_DISTANCE, end_distance);
 GetPIDController()->Enable();

}

void DriveDistance::Execute()
{
 std::printf("PID Value: %f\n", GetPIDController()->Get());
}

bool DriveDistance::IsFinished()
{
 return GetPIDController()->OnTarget();
}

void DriveDistance::End()
{

 RobotMap::driveEncoderL->Reset();
 Robot::drivetrain->SetTankDrive(0.0f, 0.0f);

 GetPIDController()->Disable();
}

void DriveDistance::Interrupted()
{
 std::printf("ERROR: DriveDistance interrupted!\n");
 Robot::drivetrain->SetTankDrive(0.0f, 0.0f);

 GetPIDController()->Disable();
}

double DriveDistance::ReturnPIDInput()
{
 return Robot::drivetrain->GetEncoderDistance();
}

void DriveDistance::UsePIDOutput(double output)
{
 Robot::drivetrain->SetTankDrive(output, output);
}
