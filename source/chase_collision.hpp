#pragma once
#include "camera.hpp"

namespace whoosh::camera {
struct ChaseBody {
    Vec centre, right, forward, up, lo, hi;
    bool valid;
    Vec local(Vec d) const { return {dot(d,right),dot(d,forward),dot(d,up)}; }
};
struct ChaseLens { float nearZ, halfWidth, halfHeight; };
struct ChaseHit { Vec point, normal; };
struct ChaseTravel {
    Vec previousPivot{}, previousEye{}, relief{};
    float radius{};
    bool limited{};
    bool valid{};
};

inline std::array<Vec,4> nearCorners(const Pose& pose,ChaseLens lens) {
    const Vec centre=pose.front*lens.nearZ;
    const Vec right=cross(pose.front,pose.up)*(lens.halfWidth*lens.nearZ);
    const Vec up=pose.up*(lens.halfHeight*lens.nearZ);
    return {centre+right+up,centre+right-up,centre-right+up,centre-right-up};
}

template<class Cast>
Pose resolveChase(Placement desired,const ChaseBody& body,ChaseLens lens,
                  ChaseTravel& travel,float dt,Cast cast) {
    const Vec pivot=desired.anchor;
    const Vec sight=desired.look-desired.eye;
    const float searchMinimum=body.valid ? std::max({.6f,-body.lo.y,body.hi.y})+lens.nearZ+.2f : .6f;
    if(length(desired.eye-pivot)<.01f) {
        Vec back=sight*(-1.f);
        if(length(back)<.01f) back=body.valid ? body.forward*(-1.f) : Vec{0,-1,0};
        desired.eye=pivot+back*(searchMinimum/length(back));
        if(desired.keepBearing) desired.look=desired.eye+sight;
    }
    const auto requestedOrientation=orient(desired,desired.eye);
    const auto footprint=nearCorners(requestedOrientation,lens);
    const float farthestCorner=length(footprint[0]);
    constexpr float skin=.045f;

    // Maintain ground clearance by changing elevation at the requested distance.
    // Retraction along a downward boom would send the camera towards the cabin.
    ChaseHit hit;
    const float probeTop=pivot.z+std::min(1.f,farthestCorner);
    // A downward probe must start on the car's side of a low ceiling.
    const float startZ=body.valid ? std::min(probeTop,body.centre.z+std::max(.4f,body.hi.z)) : probeTop;
    if(desired.eye.z-farthestCorner<startZ &&
       cast({desired.eye.x,desired.eye.y,startZ},
            {desired.eye.x,desired.eye.y,desired.eye.z-farthestCorner-skin},hit) &&
       std::abs(hit.normal.z)>.45f) {
        // Line-of-sight returns the stored triangle plane; collision assets
        // can use either winding. Orient the support plane toward the probe.
        if(hit.normal.z<0) hit.normal=hit.normal*(-1.f);
        float lowest=0;
        for(Vec corner:footprint) lowest=std::min(lowest,dot(hit.normal,corner));
        const float lift=(skin-lowest-dot(hit.normal,desired.eye-hit.point))/hit.normal.z;
        if(lift>0) {
            desired.eye.z+=lift;
            if(desired.keepBearing) desired.look=desired.eye+sight;
        }
    }

    const Vec offset=desired.eye-pivot;
    const float distance=length(offset);
    if(distance<.001f) { travel={}; return requestedOrientation; }
    const Vec outward=offset*(1.f/distance);
    Vec lo=body.lo,hi=body.hi;
    if(body.valid) {
        Vec low{},high{};
        for(Vec corner:footprint) {
            const Vec c=body.local(corner);
            low={std::min(low.x,c.x),std::min(low.y,c.y),std::min(low.z,c.z)};
            high={std::max(high.x,c.x),std::max(high.y,c.y),std::max(high.z,c.z)};
        }
        lo=lo-high;
        hi=hi-low;
    }
    auto bodyExit=[&](Vec eye,Vec direction) {
        return body.valid ? bodyClearance(body.local(eye-body.centre),body.local(direction),lo,hi,.035f) : 0.f;
    };
    const float reach=distance+bodyExit(desired.eye,outward);
    auto clearance=[&](Vec origin,Vec direction,bool preview) {
        float limit=reach;
        auto probe=[&](Vec corner) {
            const Vec end=origin+direction*(reach+skin)+corner;
            ChaseHit contact;
            if(!cast(origin,end,contact)) return;
            Vec normal=contact.normal;
            if(preview && std::abs(normal.z)>.7f) return;
            if(dot(normal,end-origin)>0) normal=normal*(-1.f);
            const float incidence=-dot(normal,direction);
            if(incidence>.025f) {
                const float stop=(dot(normal,origin+corner-contact.point)-skin)/incidence;
                limit=std::min(limit,std::max(0.f,stop));
            } else {
                const float stop=dot(contact.point-origin-corner,direction)-skin;
                limit=std::min(limit,std::max(0.f,stop));
            }
        };
        probe({});
        for(Vec corner:footprint) probe(corner);
        return limit;
    };
    const float available=clearance(pivot,outward,false);
    float radius=available;
    if(travel.valid && dt>0) {
        const Vec shift=pivot-travel.previousPivot;
        const Vec eyeShift=desired.eye-travel.previousEye;
        float target=available;
        const float speedStep=std::max(length(shift),length(eyeShift));
        if(speedStep>.001f && speedStep<4.f) {
            // Sample the upcoming edge before the current eye reaches it.
            // Prediction only affects distance; it never steers the orbit.
            const float steps=std::min(.14f/dt,2.f/speedStep);
            const Vec predicted=offset+(eyeShift-shift)*steps;
            const float d=length(predicted);
            if(d>.01f) target=std::min(target,clearance(pivot+shift*steps,predicted*(1.f/d),true));
        }
        const float rate=target<travel.radius ? 15.f : 6.f;
        if(target<reach-.001f || travel.limited)
            radius=std::min(available,travel.radius+(target-travel.radius)*(-std::expm1(-rate*dt)));
    }
    Vec eye=pivot+outward*radius;
    const float extra=bodyExit(eye,outward);
    // Prefer the requested bearing whenever the space between wall and body
    // is sufficient. Otherwise seek the nearest clear exit from the body.
    Vec targetRelief{};
    if(extra>0 && radius+extra>available) {
        const Vec side=cross(requestedOrientation.front,requestedOrientation.up);
        const Vec up{0,0,1};
        float best=1.e6f;
        for(Vec direction:{up,side,side*(-1.f),up+side,up-side,
                           body.forward,body.forward*(-1.f)}) {
            const float magnitude=length(direction);
            if(magnitude<.01f) continue;
            direction=direction*(1.f/magnitude);
            const float step=bodyExit(eye,direction)+.10f;
            const Vec delta=direction*step;
            const float cost=step+.5f*length(delta-travel.relief);
            if(cost>=best) continue;
            const Vec end=eye+delta;
            if(dot(end-pivot,outward)<0 || cast(pivot,end,hit) || cast(eye,end,hit)) continue;
            bool clear=true;
            for(Vec corner:footprint) {
                if(cast(eye+corner,end+corner,hit) || cast(end,end+corner,hit)) {
                    clear=false;
                    break;
                }
            }
            if(clear) { best=cost; targetRelief=delta; }
        }
    }
    Vec relief=travel.relief+(targetRelief-travel.relief)*(-std::expm1(-7.f*dt));
    const float size=length(relief);
    if(size<.001f) relief={};
    else {
        float fraction=1;
        auto sweep=[&](Vec start) {
            if(cast(start,start+relief,hit))
                fraction=std::min(fraction,std::clamp((dot(hit.point-start,relief)/size-skin)/size,0.f,1.f));
        };
        sweep(eye);
        for(Vec corner:footprint) sweep(eye+corner);
        relief=relief*fraction;
        eye=eye+relief;
    }
    travel={pivot,desired.eye,relief,radius,radius<reach-.01f,true};
    // The pivot rays taper towards the vehicle. At a narrow edge a corner of
    // the full-size near plane can still touch the wall. Slide locally along
    // its normal, keeping the requested orientation throughout.
    for(int pass=0;pass<2;++pass) {
        Vec correction{};
        float depth=0;
        for(Vec corner:footprint) {
            ChaseHit contact;
            if(!cast(eye,eye+corner,contact)) continue;
            Vec normal=contact.normal;
            if(dot(normal,corner)>0) normal=normal*(-1.f);
            const float overlap=skin-dot(normal,eye+corner-contact.point);
            if(overlap>depth) { depth=overlap; correction=normal*overlap; }
        }
        if(depth<=0 || cast(eye,eye+correction,hit)) break;
        eye=eye+correction;
    }
    // Retraction must not turn the camera around as it crosses the look target.
    return {eye,requestedOrientation.front,requestedOrientation.up};
}
}
