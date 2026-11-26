#pragma once
#include "detector.hpp"

namespace whoosh::visuals {
struct UV { float x,y; };
struct Projection {
    Vec x,y,z,origin;
    Vec vector(Vec v) const {return x*v.x+y*v.y+z*v.z;}
    Vec point(Vec p) const {return vector(p)+origin;}
};
// Derivative of perspective projection when the camera translates with the car.
// Sample backwards along the flow of stationary scenery.
inline UV sampleMotion(UV p,Vec viewVelocity,float exposure) {
    return {p.x+(viewVelocity.x-p.x*viewVelocity.z)*exposure,
            p.y+(viewVelocity.y-p.y*viewVelocity.z)*exposure};
}
// GTA vehicle velocity is converted to m/s by the caller.
inline float blurDrive(bool carbon,float speed,float nitro) {
    float cruise=unitRange((speed-(carbon?9.f:13.f))/(carbon?30.f:35.f));
    return cruise + unitRange(nitro)*unitRange(speed/18.f)*(carbon?.85f:.45f);
}
inline constexpr float carbonWeights[]{6.f/23,5.f/23,4.f/23,3.f/23,2.f/23,1.f/23,1.f/23,1.f/23};

// Each draw mixes a new sample into the weighted mean of all previous samples.
constexpr unsigned spatialAlpha(bool carbon,unsigned tap) {
    constexpr unsigned mw[]{1,2,2,3,3,2,2,1};
    constexpr unsigned nfsc[]{6,5,4,3,2,1,1,1};
    const auto& weights=carbon ? nfsc : mw;
    unsigned total=0;
    for(unsigned i=0;i<=tap;++i)total+=weights[i];
    return (255*weights[tap]+total/2)/total;
}

// Keep the first filter dense at the scratch resolution. Later passes
// convolve the filtered result, with progressively wider support.
inline UV spatialTap(UV uv,Vec motion,float exposure,bool carbon,unsigned pass,
                     unsigned tap,unsigned width,unsigned height) {
    const float scale=pass==1 ? .3f : pass==2 ? 1.2f : 4.8f;
    const float limit=pass==1 ? 1.f : pass==2 ? 4.f : 16.f;
    // Uniform scaling across the quad keeps the flow field affine under
    // interpolation. Its largest step is attained at a corner of the frame.
    const float dx=std::max(std::abs(motion.x),std::abs(motion.x-motion.z))*width;
    const float dy=std::max(std::abs(motion.y),std::abs(motion.y-motion.z))*height;
    const float step=std::min(scale*exposure,limit/std::max({dx,dy,1.f}));
    return sampleMotion(uv,motion,step*(carbon ? float(tap) : float(tap)-3.5f));
}
inline UV nitrousRadialTap(UV uv,float amount,unsigned tap) {
    float offset=amount*tap/7.f;
    return {uv.x+(.5f-uv.x)*offset,uv.y+(.5f-uv.y)*offset};
}
// Most Wanted visuallookeffect/nos_radialblur, slot 0x6B40EB80.
inline float mwNitro(float secondsAfterRelease) {
    float t=unitRange(1.f-secondsAfterRelease/5.f),u=1.f-t;
    return 1.5f*(3.f*t*u*u*.3947368f+3.f*t*t*u*.7631579f+t*t*t);
}
inline float smooth(float x) {x=unitRange(x);return x*x*(3.f-2.f*x);}
}
