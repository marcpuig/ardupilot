/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_IR_H__
#define __AP_RANGEFINDER_IR_H__

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_ir : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_ir(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);
};
#endif // __AP_RANGEFINDER_IR_H__

