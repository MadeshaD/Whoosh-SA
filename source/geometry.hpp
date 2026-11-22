#pragma once
#include "detector.hpp"

namespace whoosh {
inline bool overheadPanel(Vec span) {
    float depth=std::min(span.x,span.y), width=std::max(span.x,span.y);
    return depth>=0 && depth<=6.f && width>=4.5f && width>=2.4f*depth &&
           (span.z>=2.f || (depth<=1.5f && span.z>=.08f)) && width>=1.15f*span.z && span.z<=30.f;
}
// A model's local collision box and its world transform. Shared by the game
// adapter and the scene runner, including for models with an off-centre pivot.
struct Shape {
    Vec origin{},x{1,0,0},y{0,1,0},z{0,0,1},lo{},hi{};
};
inline bool bodyFrom(const Shape& s, Vec along, uintptr_t id, bool traffic, Vec velocity, Body& out) {
    Vec span=s.hi-s.lo;
    float narrow=std::min(span.x,span.y),wide=std::max(span.x,span.y);
    bool sign=!traffic && overheadPanel(span);
    if(narrow<0 || (!sign && wide>15.f) || narrow>(sign?12.f:8.5f) || (!sign && span.z<.42f) || span.z>30.f)return false;
    Vec local=(s.lo+s.hi)*.5f,half=span*.5f;
    Vec mid=s.origin+s.x*local.x+s.y*local.y+s.z*local.z;
    float hz=std::abs(s.x.z)*half.x+std::abs(s.y.z)*half.y+std::abs(s.z.z)*half.z;
    Vec right{along.y,-along.x,0};
    auto extent=[&](Vec a) {
        return std::abs(dot(s.x,a))*half.x+std::abs(dot(s.y,a))*half.y+std::abs(dot(s.z,a))*half.z;
    };
    bool upright=!sign && !traffic && span.z>2.5f && span.z>wide*.95f;
    // The origin usually lies in the stem. A lamp's arm or a tree crown
    // should not sound as if the car had passed through the whole box.
    Vec anchor=mid;
    if(upright) {
        anchor=s.origin+s.x*std::clamp(0.f,s.lo.x,s.hi.x)+s.y*std::clamp(0.f,s.lo.y,s.hi.y)+s.z*local.z;
    }
    float across=upright?std::clamp(narrow*.18f,.14f,.85f):extent(right);
    float forward=upright?across:extent(along);
    bool large=traffic || (!upright && wide>3.4f) || (upright && narrow>4.f);
    out={id,anchor,velocity,across,forward,mid.z-hz,mid.z+hz,large}; out.sign=sign;out.family=traffic?Family::Traffic:(upright || !large ? Family::Post:Family::Smack);return true;
}
}
