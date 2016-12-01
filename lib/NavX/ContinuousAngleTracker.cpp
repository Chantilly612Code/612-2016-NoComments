# 8 "./lib/NavX/ContinuousAngleTracker.cpp"
#include "ContinuousAngleTracker.h"

ContinuousAngleTracker::ContinuousAngleTracker() {
    this->last_angle = 0.0f;
    this->zero_crossing_count = 0;
    this->last_rate = 0;
    this->first_sample = false;
}

void ContinuousAngleTracker::NextAngle( float newAngle ) {







 if ( this->first_sample ) {
  this->first_sample = false;
  if ( newAngle < 0.0f ) {
   this->zero_crossing_count--;
  }
 }






 bool bottom_crossing = false;
 float delta_angle = newAngle - this->last_angle;

    if ( delta_angle >= 180.0f ){
     delta_angle = 360.0f - delta_angle;
     bottom_crossing = true;
    } else if ( delta_angle <= -180.0f ){
     delta_angle = 360.0f + delta_angle;
     bottom_crossing = true;
    }
    this->last_rate = delta_angle;




    if ( !bottom_crossing ) {
        if ( delta_angle < 0.0f ) {
         if ( (newAngle < 0.0f) && (this->last_angle >= 0.0f) ) {
          this->zero_crossing_count--;
         }
        } else if ( delta_angle >= 0.0f ) {
         if ( (newAngle >= 0.0f) && (last_angle < 0.0f) ) {
          this->zero_crossing_count++;
         }
        }
    }
    this->last_angle = newAngle;
}

double ContinuousAngleTracker::GetAngle() {
    double accumulated_angle = (double)this->zero_crossing_count * 360.0f;
    double curr_angle = (double)this->last_angle;
    if ( curr_angle < 0.0f ) {
        curr_angle += 360.0f;
    }
    accumulated_angle += curr_angle;
    return accumulated_angle;
}

double ContinuousAngleTracker::GetRate() {
    return this->last_rate;
}
