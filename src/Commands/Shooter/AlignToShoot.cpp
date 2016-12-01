# 1 "./src/Commands/Shooter/AlignToShoot.cpp"
#include "AlignToShoot.h"
#include <Commands/Autonomous/Sequences/AutoAlign.h>
#include <Commands/Autonomous/Sequences/VerticalAlign.h>
#include "SetShooterAngle.h"

AlignToShoot::AlignToShoot()
{
 SetInterruptible(true);
 AddSequential(new AutoAlign(HorizontalFind::RIGHT));
}
