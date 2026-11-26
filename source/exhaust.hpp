#pragma once
#include "detector.hpp"

namespace whoosh::visuals {
// Carbon PC emitterdata, decoded using the field offsets in attributes.bin.
// The local positive Z axis of the FX marker points out of the exhaust.
struct ExhaustEmitter {
    float life, rate, speed, gravity, lifeVariance;
    std::array<float,4> size;
    std::array<unsigned,4> color;
};
constexpr ExhaustEmitter exhaustEmitters[]={
    {.15f,25,4.5f,-10,0,{0,.05f,.2f,0},{0xffffffff,0xff6363ff,0x634426ff,0}},
    {.15f,25,4.5f,-10,0,{.1f,.1f,.65f,0},{0x1808ffff,0xffd50d7f,0x630d017f,0}},
    {.35f,40,5,-1,.125f,{0,.25f,.5f,1.25f},{0x828287ff,0x898993bf,0x93939e7f,0x7d7d7d00}}
};
// Cubic through the four equally spaced keys used by these three emitters.
inline std::array<float,4> exhaustKeyWeights(float t) {
    float u=3*t;
    return {-(u-1)*(u-2)*(u-3)/6,u*(u-2)*(u-3)/2,
            -u*(u-1)*(u-3)/2,u*(u-1)*(u-2)/6};
}
struct ExhaustPuff {
    Vec origin{},velocity{};
    float age{},life{},strength{},angle{};
    Vec position(float gravity) const {
        Vec p=origin+velocity*age;
        p.z-=.5f*gravity*age*age;
        return p;
    }
};
struct ExhaustStream {
    // At 40/s and 0.35 s lifetime there are at most 14 live particles per outlet.
    std::array<ExhaustPuff,16> puffs{};
    unsigned next{};
    float phase{};
};
inline constexpr float lampHalfSizeMetres=.3f;
}
