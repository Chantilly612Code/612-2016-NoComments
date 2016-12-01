# 1 "./src/VisionTarget.h"
#ifndef VISIONTARGET_H_
#define VISIONTARGET_H_ 

#include "WPILib.h"
#include <cmath>


class VisionTarget
{
public:
 static const int PARAM_COUNT = 4;

 static const double HEIGHT;
 static const double WIDTH;

 VisionTarget(std::vector<int> initPoints, int id);
 virtual ~VisionTarget();

 Point GetUL();
 Point GetLL();
 Point GetUR();
 Point GetLR();

 Point GetCenter();

 int GetWidth();
 int GetHeight();

 int GetHeightConvex();
 double GetDistance();

 double GetAspectRatio();

 int GetID();

 void Set(std::vector<int> setPoints);


 void Print();

 static std::shared_ptr<VisionTarget> FindClosestAspect(double aspect, std::vector<std::shared_ptr<VisionTarget>> targets);

private:
 int x;
 int y;
 int width = 0;
 int height = 0;
 int hHeight;

 int id;

 Point MakePoint(int x, int y);

};

#endif
