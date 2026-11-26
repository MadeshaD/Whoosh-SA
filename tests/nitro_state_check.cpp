#ifndef _WIN32
#define __cdecl
#endif
#include "../source/game.hpp"
#include <cassert>

int main() {
    using namespace whoosh;
    std::array<uint8_t,0x900> vehicle{};
    auto car=reinterpret_cast<uintptr_t>(vehicle.data());
    auto put=[&](unsigned offset,auto value){std::memcpy(vehicle.data()+offset,&value,sizeof value);};
    put(0x38c,0x80000u);
    put(0x428,uint8_t(0x10));
    put(0x8a4,-.5f);
    assert(nitroCycle(car) && !nitroActive(car));
    put(0x49c,1.f);
    assert(nitroActive(car));
    put(0x49c,0.f);
    assert(!nitroActive(car) && nitroCycle(car) && nitroPower(car)==.5f);
    put(0x49c,.25f);
    assert(nitroActive(car));
    put(0x8a4,1.f); // AdvancedNitroSystem release or purge
    assert(!nitroCycle(car) && !nitroActive(car) && nitroPower(car)==0);
    put(0x8a4,-.5f);
    put(0x590,9u);put(0x594,9u);
    assert(!nitroActive(car));
    bikeNitroLayout=true;
    assert(nitroActive(car));
    put(0x42b,uint8_t(0x40));
    assert(!nitroActive(car));
    assert(!nitroActive(0));
}
