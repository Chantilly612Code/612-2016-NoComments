# 1 "./src/RobotMap.h"
       
#include <WPILib.h>
#include <Ports.h>
#include <AbsoluteEncoder/AbsoluteEncoder.h>
#include <NavX/AHRS.h>







class RobotMap
{
public:
 static std::shared_ptr<CANTalon> driveTalonFL;
 static std::shared_ptr<CANTalon> driveTalonRL;
 static std::shared_ptr<CANTalon> driveTalonFR;
 static std::shared_ptr<CANTalon> driveTalonRR;
 static std::shared_ptr<Servo> shifterL;
 static std::shared_ptr<Servo> shifterR;
 static std::shared_ptr<Encoder> driveEncoderL;
 static std::shared_ptr<Encoder> driveEncoderR;
 static std::shared_ptr<RobotDrive> drivetrainRobotDrive;

 static std::shared_ptr<CANTalon> shooterActuatorMotor;
 static std::shared_ptr<AbsoluteEncoder> shooterPotentiometer;
 static std::shared_ptr<CANTalon> shooterRotateMotor;
 static std::shared_ptr<DoubleSolenoid> shooterSolenoid;

 static std::shared_ptr<CANTalon> flywheelMotorL;
 static std::shared_ptr<CANTalon> flywheelMotorR;
 static std::shared_ptr<CANTalon> armRotateMotor;

 static std::shared_ptr<AHRS> NavX;

 static constexpr float flywheelHallL = IDS::shooterHallL;
 static constexpr float flywheelHallR = IDS::shooterHallR;





 static void init();
};
