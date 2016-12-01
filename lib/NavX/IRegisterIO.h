# 8 "./lib/NavX/IRegisterIO.h"
#ifndef SRC_IREGISTERIO_H_
#define SRC_IREGISTERIO_H_ 

#include <stdint.h>

class IRegisterIO {
public:
    IRegisterIO(){}
    virtual ~IRegisterIO() {}
    virtual bool Init() = 0;
    virtual bool Write(uint8_t address, uint8_t value ) = 0;
    virtual bool Read(uint8_t first_address, uint8_t* buffer, uint8_t buffer_len) = 0;
    virtual bool Shutdown() = 0;
};

#endif
