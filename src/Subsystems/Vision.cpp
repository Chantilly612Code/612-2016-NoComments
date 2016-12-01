# 1 "./src/Subsystems/Vision.cpp"
#include "Vision.h"
#include "../RobotMap.h"
#include "Robot.h"

Vision::Vision() :
  Subsystem("Vision")
{
 NetworkTable::SetPort(1735);
 table = NetworkTable::GetTable("Vision");
 goal = NULL;
}

void Vision::InitDefaultCommand() { }

void Vision::PullValues()
{




 llvm::ArrayRef<double> arr;
 std::vector<double> coords = table->GetNumberArray(BOUNDING_KEY, arr);

 llvm::ArrayRef<double> arr2;
 std::vector<double> ids = table->GetNumberArray(IDS_KEY, arr2);


 int targetCount = std::min(ids.size(), coords.size() / VisionTarget::PARAM_COUNT);


 for (int x = 0; x < targetCount; x++)
 {
  std::vector<int> vec;
  for (int i = 0; i < VisionTarget::PARAM_COUNT; i++)
  {
   if(!(coords.empty()))
    vec.push_back((int) coords[(x * VisionTarget::PARAM_COUNT) + i]);
  }

  if (TargetExists(ids[x]))
  {
   GetTargetById(ids[x])->Set(vec);
  }

  else
   targets.push_back(
     std::shared_ptr<VisionTarget>(
       new VisionTarget(vec, ids[x])));
 }


 for (uint32_t x = 0; x < targets.size(); x++)
 {
  bool found = false;
  for (uint32_t i = 0; i < ids.size(); i++)
  {
   if (targets[x]->GetID() == ids[i])
   {
    found = true;
    break;
   }
  }

  if (!found)
  {
   targets.erase(targets.begin() + x);
   x--;
  }
 }
}






std::vector<std::shared_ptr<VisionTarget>> Vision::GetAllTargets()
{
 return targets;
}

std::shared_ptr<VisionTarget> Vision::GetTargetById(int id)
{

 for (uint32_t x = 0; x < targets.size(); x++)
 {
  if (targets[x]->GetID() == id)
   return targets[x];
 }

 return NULL;
}

bool Vision::TargetExists(int id)
{


 for (uint32_t x = 0; x < targets.size(); x++)
 {
  if (targets[x]->GetID() == id)
   return true;
 }

 return false;
}

bool Vision::TargetExists(std::shared_ptr<VisionTarget> target)
{
 return TargetExists(target->GetID());
}

std::shared_ptr<NetworkTable> Vision::GetRawTable()
{
 return table;
}

int Vision::GetTargetAmount()
{
 return targets.size();
}

bool Vision::UpdateCurrentTarget()
{
 llvm::ArrayRef<double> arr;

 if (GetTargetAmount() > 0)
 {
  goal = VisionTarget::FindClosestAspect(TARGET_ASPECT, GetAllTargets());

  return true;
 }
 else
 {
  std::printf("Warning: No vision target\n");
  return false;
 }
}

std::shared_ptr<VisionTarget> Vision::GetTrackedGoal()
{
 if (goal == NULL || !TargetExists(goal->GetID()))
 {
  UpdateCurrentTarget();
 }

 return goal;
}

void Vision::SetTrackedGoal(std::shared_ptr<VisionTarget> goal)
{
 this->goal = goal;
}

void Vision::SetTrackingID(int id)
{
 std::shared_ptr<VisionTarget> target = GetTargetById(id);
 if (target == NULL)
 {


  UpdateCurrentTarget();
 }
 else
 {
  goal = target;
 }
}

int Vision::GetTrackingID()
{
 UpdateCurrentTarget();
 if (goal != NULL)
  return goal->GetID();
 else
  return -1;
}
