// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#pragma once

#include "ord/math.hpp"

namespace ord {

struct RigidBodyState {
    Vec3 position;           // world position of the body origin
    Quat orientation;        // body -> world
    Vec3 velocity;           // world, of the centre of mass
    Vec3 angularVelocity;    // world
};

}  // namespace ord
