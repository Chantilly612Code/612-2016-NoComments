# 8 "./lib/NavX/InertialDataIntegrator.h"
#ifndef SRC_INERTIALDATAINTEGRATOR_H_
#define SRC_INERTIALDATAINTEGRATOR_H_ 

class InertialDataIntegrator {
    float last_velocity[2];
    float displacement[2];

public:
    InertialDataIntegrator();
    void UpdateDisplacement( float accel_x_g, float accel_y_g,
                             int update_rate_hz, bool is_moving );
    void ResetDisplacement();
    float GetVelocityX();
    float GetVelocityY();
    float GetVelocityZ();
    float GetDisplacementX();
    float GetDisplacementY();
    float GetDisplacementZ() ;
};

#endif
