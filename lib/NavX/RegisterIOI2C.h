# 8 "./lib/NavX/RegisterIOI2C.h"
#ifndef SRC_REGISTERIOI2C_H_
#define SRC_REGISTERIOI2C_H_ 

#include "RegisterIO.h"
#include <WPILib.h>

class RegisterIO_I2C : public IRegisterIO {
public:
    RegisterIO_I2C(I2C *port);
    virtual ~RegisterIO_I2C() {}
    bool Init();
    bool Write(uint8_t address, uint8_t value );
    bool Read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len);
    bool Shutdown();
private:
    I2C *port;
};

#endif
