# 1 "./lib/AbsoluteEncoder/AbsoluteEncoder.h"
#include <WPILib.h>
#include <AnalogInput.h>

       

class AbsoluteEncoder : public AnalogInput
{
public:
    AbsoluteEncoder(int channel);
    ~AbsoluteEncoder();
 double GetVoltageRound();
 double PIDGet();
};
