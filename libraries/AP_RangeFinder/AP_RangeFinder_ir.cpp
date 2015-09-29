// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   AP_RangeFinder_ir.cpp - rangefinder for ir source
 *
 */

#include <IPC/IPC.h>
#include "RangeFinder.h"
#include "AP_RangeFinder_ir.h"

extern IPC *ipc;

AP_RangeFinder_ir::AP_RangeFinder_ir(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
        set_status(RangeFinder::RangeFinder_NoData);
}

/*
  update distance_cm 
 */
void AP_RangeFinder_ir::update(void)
{
    float altitude;
    
    if (ipc->getAltitude(altitude)) {
        state.distance_cm = altitude * 100.f;
        update_status();
    }
    else
        set_status(RangeFinder::RangeFinder_NoData);
}

