# 1 "./src/OI.h"
       

#include "WPILib.h"

class OI
{
private:
 std::shared_ptr<Joystick> gunner;
 std::shared_ptr<JoystickButton> shoot;
 std::shared_ptr<JoystickButton> align;
 std::shared_ptr<JoystickButton> shooterHome;
 std::shared_ptr<JoystickButton> shooterIntake;

 std::shared_ptr<Joystick> driver;
 std::shared_ptr<JoystickButton> reverseControls;
 std::shared_ptr<JoystickButton> lowGear;
 std::shared_ptr<JoystickButton> highGear;

public:
 OI();

 enum Stick {DRIVER, GUNNER};
 enum RumbleSide {LEFT, RIGHT, BOTH};

 std::shared_ptr<Joystick> getDriver();
 std::shared_ptr<Joystick> getGunner();

 std::shared_ptr<JoystickButton> getButton(uint32_t button);
};
