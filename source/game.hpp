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
inline bool gameplay() {
    return !read<uint8_t>(0xBA67A4) && !read<uint8_t>(0xB7CB48) &&
           !read<uint8_t>(0xB7CB49) && !read<uint8_t>(0xB5F851);
}
}
