#pragma once
#include "detector.hpp"
#include <cstring>

namespace whoosh {
template<class T> T read(uintptr_t p) {
    T v;
    std::memcpy(&v,reinterpret_cast<const void*>(p),sizeof(v));
    return v;
}
struct ColPoint { Vec position; float pad; Vec normal; uint8_t rest[16]; };
static_assert(sizeof(ColPoint)==44);
using Trace=bool(__cdecl*)(const Vec&,const Vec&,ColPoint&,uintptr_t&,
                          bool,bool,bool,bool,bool,bool,bool,bool);
// Enabled only when the installed bike extension owns the exhaust dispatch.
inline bool bikeNitroLayout{};
inline bool nitroVehicle(uintptr_t car) {
    return car && (read<unsigned>(car+0x590)==0 ||
                   (bikeNitroLayout && read<unsigned>(car+0x594)==9));
}
inline bool nitroCycle(uintptr_t car) {
    return nitroVehicle(car) && (read<unsigned>(car+0x38c)&0x80000u) &&
           read<float>(car+0x8a4)<0;
}
inline bool nitroActive(uintptr_t car) {
    return nitroCycle(car) && read<float>(car+0x49c)>0.f &&
           (read<uint8_t>(car+0x428)&0x10) && !(read<uint8_t>(car+0x42b)&0x40);
}
// GTA keeps feeding the exhaust at half power during the active charge on lift-off.
inline float nitroPower(uintptr_t car) {
    if(!nitroCycle(car) || !(read<uint8_t>(car+0x428)&0x10) || (read<uint8_t>(car+0x42b)&0x40))return 0;
    return .5f+.5f*unitRange(read<float>(car+0x49c));
}
inline bool gameplay() {
    return !read<uint8_t>(0xBA67A4) && !read<uint8_t>(0xB7CB48) &&
           !read<uint8_t>(0xB7CB49) && !read<uint8_t>(0xB5F851);
}
}
