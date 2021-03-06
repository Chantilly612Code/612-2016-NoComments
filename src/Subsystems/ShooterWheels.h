# 1 "./src/Subsystems/ShooterWheels.h"
       

#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "RobotMap.h"
#include "HallEffect/PIDEdgeCounter.h"
#include <PIDController.h>
#include "HallEffect/NetworkTables.h"

class ShooterWheels: public Subsystem
{
private:

    const float kP = 1.5f;
    const float kI = .4f;
    const float kD = .4f;
    const float kTol = 0.05f;
 std::shared_ptr<CANTalon> CANTalonLeft;
 std::shared_ptr<CANTalon> CANTalonRight;
 std::shared_ptr<PIDEdgeCounter> hallCounterLeft;
 std::shared_ptr<PIDEdgeCounter> hallCounterRight;
 std::shared_ptr<PIDController> wheelControllerLeft;
 std::shared_ptr<PIDController> wheelControllerRight;

 float leftLastErr = 999.9f;
 float rightLastErr = 999.9f;
 bool manualstarted = false;
public:
 ShooterWheels();
 float GetLeftWheelSpeed();
 float GetRightWheelSpeed();
 bool UpToSpeed();
 void SetWheelSpeed(float speed);
 void Disable();
 void Enable();
 void InitDefaultCommand();
 void ManualSet(float speed);
 void StartManual();
 NetworkTables* shootertable;
 bool enabled = false;
};
