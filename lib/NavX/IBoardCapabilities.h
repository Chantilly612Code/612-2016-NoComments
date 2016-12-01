# 8 "./lib/NavX/IBoardCapabilities.h"
#ifndef SRC_IBOARDCAPABILITIES_H_
#define SRC_IBOARDCAPABILITIES_H_ 

class IBoardCapabilities {
public:
    IBoardCapabilities() {}
    virtual bool IsOmniMountSupported() = 0;
    virtual bool IsBoardYawResetSupported() = 0;
    virtual bool IsDisplacementSupported() = 0;
};

#endif
