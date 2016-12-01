# 1 "./src/OI.cpp"
#include "OI.h"
#include "Commands/Drive/DriveJoystick.h"
#include "Commands/Drive/SetGear.h"
#include "Commands/Shooter/AlignToShoot.h"
#include "Commands/Shooter/SetShooterAngle.h"
#include <Commands/Shooter/Shoot.h>

OI::OI()
{

 gunner.reset(new Joystick(1));

 shoot.reset(new JoystickButton(gunner.get(), 1));
 shoot.get()->WhenPressed(new Shoot(true));
 shoot.get()->WhenReleased(new Shoot(false));

 align.reset(new JoystickButton(gunner.get(), 4));


 driver.reset(new Joystick(0));

 lowGear.reset(new JoystickButton(driver.get(), 5));
 lowGear->WhenPressed(new SetGear(Shifter::LOW));



}

std::shared_ptr<Joystick> OI::getDriver()
{
 return driver;
}

std::shared_ptr<Joystick> OI::getGunner()
{
 return gunner;
}
