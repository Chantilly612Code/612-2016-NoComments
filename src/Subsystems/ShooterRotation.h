# 1 "./src/Subsystems/ShooterRotation.h"
       


#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <CANTalon.h>
#include <AbsoluteEncoder/AbsoluteEncoder.h>
#include <PIDControl/PIDControl.h>
#include <cmath>

class ShooterRotation : public Subsystem
{
private:
 double PositionToVolts(double angle);

 std::shared_ptr<CANTalon> RotateMotor;

 double AngleToVolts(double angle) {return (angle * V_OVER_A) + zerodegrees + BIAS;}

public:
 const double MAX_ANGLE = 190.0;
 const double MIN_ANGLE = 0;


 std::shared_ptr<CANTalon> motor;
 PIDControl* pid;
 AbsoluteEncoder* absEncoder;


 float kP = 20.0f;
 float kI = 0.0f;
 float kD = 0.0f;
 float gain_switch = 0.6f;


 const float zerodegrees = 1.704;
 const float oneeightydegrees = 2.095;

 const float INTAKE_ANGLE = 190;
 const float HOME_ANGLE = 0;
 const float LOGOAL_ANGLE = 180;
 const float HIGOAL_ANGLE = 55;

 const float V_OVER_A = (std::abs(oneeightydegrees - zerodegrees) / 180.0);
 const float BIAS = 0;

 void SetSetpoint(float set);

public:
 ShooterRotation();
 void SetAngle(double pos);

 void InitDefaultCommand();


 void Gun(float gunner_axis);


 void HomePos();
 void ShootPos(float position);
 void IntakePos();

 void Stop();
 bool PIDEnabled() { return pid->IsEnabled(); }
 void PIDEnable(bool enabled);

 void SmartDashboardOutput();

 bool OnTarget();


};
