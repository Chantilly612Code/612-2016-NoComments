# 1 "./lib/HallEffect/NetworkTables.h"
#ifndef SRC_HALLEFFECT_NETWORKTABLES_H_
#define SRC_HALLEFFECT_NETWORKTABLES_H_ 

#include <chrono>

#include <WPILib.h>


class NetworkTables
{
public:
 NetworkTables();

 void AddValue(double val);

private:
 std::shared_ptr<NetworkTable> wheels;
 std::chrono::high_resolution_clock::time_point startTime;
};

#endif
