#pragma once
#include "camera.hpp"

namespace whoosh::camera {
void installGame();
bool active();
void select(Profile p,float factor,VehicleOptions vehicles={},float timeout=3,float speed=2,bool hudShake=true,RoofOptions roof={});
}
