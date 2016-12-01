# 1 "./src/Commands/Autonomous/Sequences/AutoAlign.cpp"
#include "AutoAlign.h"

AutoAlign::AutoAlign(HorizontalFind::Direction dir)
{
 AddSequential(new HorizontalFind(dir));
 AddSequential(new HorizontalAlign());
 AddParallel(new VerticalAlign());
}
