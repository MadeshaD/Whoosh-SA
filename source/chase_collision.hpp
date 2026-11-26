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
    Vec offset{};
    float radius{};
    bool valid{}, constrained{};
};

inline std::array<Vec,4> nearCorners(const Pose& pose,ChaseLens lens) {
    const Vec centre=pose.front*lens.nearZ;
    const Vec right=cross(pose.front,pose.up)*(lens.halfWidth*lens.nearZ);
    const Vec up=pose.up*(lens.halfHeight*lens.nearZ);
    return {centre+right+up,centre+right-up,centre-right+up,centre-right-up};
}

inline float bodyExit(const ChaseBody& body,Vec eye,Vec outward,const std::array<Vec,4>& corners) {
    if(!body.valid) return 0;
    Vec low{},high{};
    for(Vec corner:corners) {
        const Vec p=body.local(corner);
        low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};
        high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)};
    }
    return bodyClearance(body.local(eye-body.centre),body.local(outward),
                         body.lo-high,body.hi-low,.035f);
}

template<class Cast>
Pose resolveChase(Placement desired,const ChaseBody& body,ChaseLens lens,
                  ChaseTravel& travel,float dt,Cast cast) {
    const Vec pivot=desired.anchor;
    const Vec sight=desired.look-desired.eye;
    const float sightLength=length(sight);
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

    const Vec wantedOffset=desired.eye-pivot;
    const float requestPitch=std::atan2(wantedOffset.z,std::hypot(wantedOffset.x,wantedOffset.y));
    const float requestYaw=std::atan2(wantedOffset.x,wantedOffset.y);
    const float lookYaw=std::atan2(sight.x,sight.y);
    const float lookPitch=std::atan2(sight.z,std::hypot(sight.x,sight.y));
    auto clip=[&](Vec wanted) {
        Vec radial=wanted-pivot;
        float distance=length(radial);
        if(distance<.001f) return orient(desired,pivot);
        const Vec outward=radial*(1.f/distance);
        Placement aim=desired;
        aim.eye=wanted;
        if(aim.keepBearing) {
            const float yaw=lookYaw+std::atan2(radial.x,radial.y)-requestYaw;
            const float pitch=std::clamp(lookPitch+requestPitch-std::atan2(radial.z,std::hypot(radial.x,radial.y)),-1.55f,1.55f);
            const Vec front{std::sin(yaw)*std::cos(pitch),std::cos(yaw)*std::cos(pitch),std::sin(pitch)};
            aim.look=wanted+front*sightLength;
        }
        auto corners=nearCorners(orient(aim,wanted),lens);
        distance+=bodyExit(body,wanted,outward,corners);
        wanted=pivot+outward*distance;
        float allowed=distance;
        auto stopAt=[&](Vec corner) {
            const Vec end=wanted+corner+outward*skin;
            ChaseHit contact;
            if(!cast(pivot,end,contact)) return;
            const Vec line=end-pivot;
            const float lineLength2=dot(line,line);
            if(lineLength2<1.e-8f) return;
            const float t=std::clamp(dot(contact.point-pivot,line)/lineLength2,0.f,1.f);
            // As the eye retracts the near plane keeps its physical size.
            // Project that size onto the actual face, including grazing hits.
            Vec normal=contact.normal;
            if(dot(normal,line)>0) normal=normal*(-1);
            const float incidence=-dot(normal,outward);
            float stop=t*distance-skin;
            if(incidence>.025f)
                stop=(dot(normal,pivot+corner-contact.point)-skin)/incidence;
            allowed=std::min(allowed,std::max(0.f,stop));
        };
        stopAt({});
        for(Vec corner:corners) stopAt(corner);
        const Vec eye=pivot+outward*allowed;
        // Don't let an obstruction reverse the viewing direction as the eye
        // approaches or crosses a motion-shifted look target.
        if(dot(aim.look-eye,sight)<.05f*sightLength) aim.keepBearing=true;
        return orient(aim,eye);
    };
    auto valid=[&](const Pose& pose) {
        const Vec d=pose.position-pivot;
        const float lengthD=length(d);
        if(lengthD<.05f) return false;
        const auto corners=nearCorners(pose,lens);
        if(bodyExit(body,pose.position,d*(1.f/lengthD),corners)>0) return false;
        // Corner rays from a common pivot taper toward the car. At a clipped
        // endpoint recheck the actual eye-to-plane footprint at full size.
        for(Vec corner:corners) {
            ChaseHit contact;
            if(cast(pose.position,pose.position+corner,contact)) return false;
        }
        return true;
    };
    Pose result=clip(desired.eye);
    const bool clipped=length(result.position-desired.eye)>.035f;
    bool usable=valid(result);
    auto onOrbit=[&](Vec basis,float yaw,float pitch,float reach,Pose& out) {
        const float azimuth=std::atan2(basis.x,basis.y)+yaw;
        const float elevation=std::clamp(std::atan2(basis.z,std::hypot(basis.x,basis.y))+pitch,-1.48f,1.48f);
        const Vec along{std::sin(azimuth)*std::cos(elevation),std::cos(azimuth)*std::cos(elevation),std::sin(elevation)};
        Pose candidate=clip(pivot+along*reach);
        if(!valid(candidate)) return false;
        out=candidate;
        return true;
    };

    // Continue from the displayed side of a wall when there is no room
    // between it and the vehicle. Don't jump to the opposite end of the car
    // just because a longer clear boom has become available there.
    if(travel.valid && (clipped || travel.constrained || !usable)) {
        const Vec end=usable ? result.position-pivot : wantedOffset;
        const float dist=length(end);
        if(dist>.01f) {
            const Vec old=travel.offset*(1.f/travel.radius), next=end*(1.f/dist);
            const float turn=std::acos(std::clamp(dot(old,next),-1.f,1.f));
            float reach=dist;
            const bool suddenRetraction=travel.radius-dist>.35f+12.f*dt;
            if(dist>travel.radius+.01f || suddenRetraction)
                reach=travel.radius+(dist-travel.radius)*(-std::expm1(-10.f*dt));
            const float weight=turn>.001f ? std::min(1.f,5.5f*dt/turn) : 1.f;
            if(!usable || weight<1.f || std::abs(reach-dist)>.01f) {
                const Vec dir=blendBearing(old,next,weight);
                const float yawDiff=std::remainder(std::atan2(next.x,next.y)-std::atan2(old.x,old.y),6.2831853f);
                const float step=std::copysign(std::clamp(dt*5.5f,.04f,.18f),yawDiff);
                // A previous, retracted radius can lie right on the body
                // margin. Search outward on the same side, then around it.
                Pose candidate;
                auto continuous=[&](Vec bearing,float yaw,float pitch,float radius) {
                    if(!onOrbit(bearing,yaw,pitch,radius,candidate)) return false;
                    return !suddenRetraction || !usable ||
                           length(candidate.position-pivot)>=reach-.15f;
                };
                if(continuous(dir,0,0,reach) || continuous(old,0,0,std::max(reach,travel.radius)) ||
                   continuous(old,step,.08f,reach) || continuous(old,-step,.08f,reach) ||
                   continuous(old,0,.21f,reach) || continuous(old,0,-.14f,reach) ||
                   continuous(old,-2*step,0,reach) || continuous(old,2*step,0,reach)) {
                    result=candidate;
                    usable=true;
                }
            }
        }
    }
    if(!usable) {
        // First frame or a moving obstacle has taken the previous spot.
        // Try the requested bearing at a different elevation before moving sideways.
        constexpr float radians=.01745329252f;
        const float sign=travel.valid && travel.offset.x*wantedOffset.y-travel.offset.y*wantedOffset.x<0 ? -1.f : 1.f;
        constexpr float alternatives[][2]={{0,14},{0,34},{0,-16},{20,5},{-20,5},{45,14},
                                           {-45,14},{75,5},{-75,5},{110,15},{-110,15},{165,10}};
        const float reach=std::max(length(wantedOffset),searchMinimum);
        for(const auto& offset:alternatives)
            if(onOrbit(wantedOffset,offset[0]*radians*sign,offset[1]*radians,reach,result)) {
                usable=true;
                break;
            }
        // At entry the native eye can be directly under a roof pivot. Varying
        // yaw on that almost vertical axis cannot reach a clear horizontal view.
        if(!usable && body.valid) {
            const Vec rear{-body.forward.x,-body.forward.y,0};
            for(float yaw:{0.f,90.f,-90.f,180.f}) {
                if(onOrbit(rear,yaw*radians,.06f,reach,result)) { usable=true; break; }
            }
        }
    }
    travel.offset=result.position-pivot;
    travel.radius=length(travel.offset);
    travel.constrained=clipped || length(result.position-desired.eye)>.045f;
    travel.valid=usable && travel.radius>.05f;
    return result;
}
}
