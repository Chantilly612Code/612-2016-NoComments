# 1 "./src/Subsystems/Vision.h"
       

#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "../VisionTarget.h"
#include <vector>
#include "DigitalOutput.h"

class Vision: public Subsystem
{
private:
 const std::string BOUNDING_KEY = "BOUNDING_COORDINATES";
 const std::string IDS_KEY = "IDS";

 std::shared_ptr<NetworkTable> table = NULL;
 std::vector<std::shared_ptr<VisionTarget>> targets;

 std::shared_ptr<VisionTarget> goal;

 const float TARGET_ASPECT = 1.66/1.00;

public:
 Vision();
 void InitDefaultCommand();
 void PullValues();


 std::vector<std::shared_ptr<VisionTarget>> GetAllTargets();
 int GetTargetAmount();
 std::shared_ptr<VisionTarget> GetTargetById(int id);
 bool TargetExists(int id);
 bool TargetExists(std::shared_ptr<VisionTarget> target);





 std::shared_ptr<NetworkTable> GetRawTable();

 bool UpdateCurrentTarget();
 std::shared_ptr<VisionTarget> GetTrackedGoal();
 void SetTrackedGoal(std::shared_ptr<VisionTarget>);
 void SetTrackingID(int id);
 int GetTrackingID();
};
