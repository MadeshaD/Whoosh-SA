#include "camera.hpp"

namespace whoosh::camera {
namespace {
float mix(float a, float b, float t) { return a + (b-a)*t; }
Vec normalized(Vec v, Vec fallback) {
    const float m=length(v);
    return m>1.e-5f && std::isfinite(m) ? v*(1.f/m) : fallback;
}
float yawOf(Vec v) { return std::atan2(v.x,v.y); }
float pitchOf(Vec v) { return std::asin(std::clamp(v.z,-1.f,1.f)); }
Vec fromAngles(float yaw,float pitch) {
    const float c=std::cos(pitch);
    return {c*std::sin(yaw),c*std::cos(yaw),std::sin(pitch)};
}
float accelerationCurve(float a) {
    // Linear segments of the five-point CameraAccelCurve, in m/s².
    const float x=std::min(std::abs(a),30.f);
    return std::copysign(x<=15.f ? .2f*x : 3.f+1.8f*(x-15.f),a);
}
float extension(Profile profile, float ratio) {
    const float values[]{1,1,1.02f,profile==Profile::Carbon ? 1.13f : 1.125f,1.2f};
    float x=unitRange(ratio)*4;
    unsigned i=std::min(static_cast<unsigned>(x),3u);
    return mix(values[i],values[i+1],x-float(i));
}
float envelope(float t) {
    if(t>=1) return 0;
    if(t<.1f) return 1-3*t;
    if(t<.3f) return .7f-(t-.1f);
    if(t<.5f) return .5f-.5f*(t-.3f);
    return .4f-.8f*(t-.5f);
}
float noiseLattice(uint32_t coordinate) {
    const uint32_t n=(coordinate<<13)^coordinate;
    const uint32_t h=(n*(n*n*15731u+789221u)+1376312589u)&0x7fffffff;
    return 1.f-float(h)/1073741824.f;
}
float noise(double phase) {
    float sum=0, amplitude=1;
    for(unsigned octave=0;octave<6;++octave) {
        const double cell=std::floor(phase);
        const auto i=static_cast<uint32_t>(static_cast<uint64_t>(cell));
        sum+=amplitude*mix(noiseLattice(i),noiseLattice(i+1),float(phase-cell));
        phase*=2;
        amplitude*=.5f;
    }
    return sum;
}
// Rows are the four scalar tracks at 0, 20, 40, 60 and 80 m/s.
// By column: chassis gain, surface gain, chassis rate, surface rate.
constexpr float roadTrack[2][5][4]={
    {{0,0,.6f,1},{.005f,.5f,1.1f,1.1f},{.03f,.8f,1.2f,1.2f},
     {.07f,1,1.3f,1.3f},{.02f,1,1,1.4f}},
    {{0,0,.8f,1.5f},{.005f,.5f,1.2f,2},{.03f,.8f,1.3f,2.1f},
     {.10f,1,1.4f,2.2f},{.05f,1,1.2f,2.3f}}
};
float roadVibration(const Input& in, double time, float speed, float acceleration, Response& out) {
    const bool carbon=in.profile==Profile::Carbon;
    const float t=std::clamp(speed/20.f,0.f,4.f);
    const unsigned row=std::min(static_cast<unsigned>(t),3u);
    const auto& table=roadTrack[carbon ? 1 : 0];
    auto track=[&](unsigned i){return mix(table[row][i],table[row+1][i],t-row);};
    const float throttle=speed>5 ? unitRange(acceleration/20.f) : 0;
    float chassis=track(0)+.15f*throttle;
    float rate=mix(track(2),.5f,throttle);
    if(speed>5 && (in.nitro || in.braking || in.handbrake)) {
        chassis=.3f;
        rate=(in.braking || in.handbrake) ? 5.f : 2.f;
    }
    const float surface=in.grounded ? track(1)*std::max(in.road.amplitude,.5f*unitRange(in.roadPulse)) : 0.f;
    if(chassis==0 && surface==0) return 0;
    constexpr float frequencies[][4]={{1.8f,2,2.125f,2},{3,5,7,5.5f}};
    constexpr float gains[][2][4]={
        {{.03f,.025f,.68f,.28f},{.007f,.01f,.3f,.4f}},
        {{.03f,.075f,.8f,.4f},{.014f,.02f,.6f,.8f}}
    };
    const float surfaceRate=track(3)*(in.road.frequency>0 ? in.road.frequency : 1.f);
    float magnitude2=0;
    for(unsigned i=0;i<4;++i) {
        float sample=0;
        if(chassis>0) sample=chassis*gains[carbon][0][i]*noise(time*rate*frequencies[0][i]);
        if(surface>0) sample+=surface*gains[carbon][1][i]*noise(time*surfaceRate*frequencies[1][i]);
        if(in.zoom==0) sample*=.25f;
        magnitude2+=sample*sample;
        out.vibration[i]=sample;
    }
    constexpr float rad=.01745329252f;
    const float fov=drawFov(in.profile,in.zoom ? 18 : 16,in.zoom,70.f);
    const float fovGain=std::sin(fov*.5f*rad)/std::sin(30.f*rad);
    for(unsigned i=2;i<4;++i)
        out.vibration[i]=std::asin(std::clamp(std::sin(out.vibration[i]*rad)*fovGain,-1.f,1.f));
    return std::sqrt(magnitude2);
}

}

Road wheelSurface(unsigned surface, unsigned roughness) {
    // Map GTA contacts to the two CAMERA_NOISE values in NFS simsurface.
    if(surface==16) return {}; // smooth golf green
    if(surface==15 || surface==6 || (surface>=24 && surface<=37)) return {.5f,1.f};
    if(surface==2 || surface==3 || surface==5 || surface==7) return {.3f,2.f};
    if((surface>=9 && surface<=14) || surface==17 || surface==20 ||
       (surface>=115 && surface<=122) || (surface>=146 && surface<=153)) return {.1f,.5f};
    if(roughness>=2) return {.5f,1.f};
    if(roughness==1) return {.25f,1.f};
    return {};
}
Vec cross(Vec a, Vec b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
Vec blendBearing(Vec from,Vec to,float weight) {
    const float yaw=yawOf(from)+weight*std::remainder(yawOf(to)-yawOf(from),6.28318530718f);
    return fromAngles(yaw,mix(pitchOf(from),pitchOf(to),weight));
}
bool supported(unsigned mode, unsigned zoom) {
    return (mode==16 && zoom==0) || ((mode==18 || mode==22) && zoom>=1 && zoom<=3);
}
float drawFov(Profile profile, unsigned mode, unsigned zoom, float nativeFov) {
    if(profile==Profile::Native || !supported(mode,zoom)) return nativeFov;
    if(!zoom) return 83;
    if(profile==Profile::Carbon) return 71;
    return zoom==3 ? 90.f : 78.f;
}
float Controller::Cubic::advance(float goal, float time, float dt) {
    // The original mover advances last frame's Hermite segment before setting
    // the next destination. Its derivative is measured in segment time.
    const float t=unitRange(dt/time);
    const float d=next-value, a=slope-2*d, b=3*d-2*slope;
    value+=t*(slope+t*(b+t*a));
    slope+=t*(2*b+3*a*t);
    next=goal;
    return value;
}
Response Controller::step(const Input& in) {
    Response out;
    if(in.profile==Profile::Native || in.zoom>3 || !(in.dt>0 && in.dt<=.15f) ||
       !std::isfinite(dot(in.velocity,in.velocity)) || !std::isfinite(dot(in.forward,in.forward)) ||
       !std::isfinite(dot(in.viewFront,in.viewFront)) || !std::isfinite(in.topSpeed) ||
       !std::isfinite(in.road.amplitude) || !std::isfinite(in.road.frequency) || !std::isfinite(in.roadPulse)) {
        *this={};
        return out;
    }
    const bool first=!started || in.profile!=lastProfile || in.zoom!=lastZoom;
    if(first) {
        *this={};
        started=true;
        lastProfile=in.profile;
        lastZoom=in.zoom;
        lastVelocity=in.velocity;
    }
    Vec accel=(in.velocity-lastVelocity)*(1/in.dt);
    lastVelocity=in.velocity;
    phase=std::min(1.f,phase+in.dt/duration);
    const Vec biased{.7f*accel.x,.7f*accel.y,accel.z};
    const float magnitude=length(biased);
    if(in.contact && magnitude>=100.f && magnitude*in.dt>=2.8f) {
        const float strength=unitRange((magnitude-100)/500);
        const float amplitude=.4f*strength;
        if(amplitude>length(shakeDirection)*envelope(phase)) {
            shakeDirection=biased*(-amplitude/magnitude);
            duration=.3f+.4f*strength;
            phase=0;
        }
    }
    if((int(phase*11.74265f)&1)==0) out.shake=shakeDirection*envelope(phase);

    const float speed=length(in.velocity);
    roadClock+=in.dt;
    const float roadLevel=roadVibration(in,roadClock,speed,dot(accel,in.forward),out);
    if(in.profile==Profile::Carbon) {
        float size=length(out.shake)*.2f;
        // Stat 2001 returns the raw speedometer value (m/s). Stat 2000
        // is the separate display-units variant. Only this fallback has a gate.
        if(size==0 && speed>=45.f) size=.05f*roadLevel;
        out.hudScale+=size;
    }
    if(in.zoom==0) {
        // GTA supplies the bumper origin for the current vehicle.
        // The original bumper looks one metre ahead and tan(1.5 degrees) down.
        out.targetHeight=-std::tan(1.5f*.01745329252f);
        return out;
    }

    samples[cursor]={accelerationCurve(accel.x),accelerationCurve(accel.y),accelerationCurve(accel.z)};
    cursor=(cursor+1)%samples.size();
    count=std::min<unsigned>(count+1,samples.size());
    Vec avg{};
    for(unsigned i=0;i<count;++i) avg=avg+samples[i];
    avg=avg*(1.f/count);

    // The two NFS chase presets bracket GTA's extra middle zoom.
    const float far=float(in.zoom-1)*.5f;
    const bool carbon=in.profile==Profile::Carbon;
    const float speedMix=unitRange(speed/71.f);
    const float baseLag=mix(-4.7f,carbon ? -5.9f : -5.436f,far);
    const float scale=in.topSpeed>0 ? extension(in.profile,speed/in.topSpeed) : 1;
    out.lag=baseLag;
    out.height=mix(1.62f,1.7f,far);
    // The driving mover blends the nose and travel directions from 8 to
    // 13 m/s. Reflect the longitudinal component in reverse before the blend.
    const float along=dot(in.velocity,in.forward);
    Vec travel=in.velocity-in.forward*(2.f*std::min(0.f,along));
    const float blend=unitRange((speed-8.f)/5.f);
    Vec desired=in.forward*(1-blend)+normalized(travel,in.forward)*blend;
    if(in.grounded) desired.z=0;
    desired=normalized(desired,in.forward);
    const Vec native=normalized(in.viewFront,desired);
    const float headingTime=mix(.4f,.2f,speedMix);
    desired=fromAngles(yawOf(desired),std::clamp(pitchOf(native),-1.39f,1.39f));
    if(first) {
        headingX={desired.x,0,desired.x};
        headingY={desired.y,0,desired.y};
        headingZ={desired.z,0,desired.z};
    }
    Vec heading{headingX.advance(desired.x,headingTime,in.dt),
                headingY.advance(desired.y,headingTime,in.dt),
                headingZ.advance(desired.z,headingTime,in.dt)};
    heading=normalized(heading,desired);
    // Keep a world-space orbit. Use GTA's processed angular deltas so its
    // sensitivity and inversion also apply to the driving profiles.
    if(in.manualLook) {
        returning=false;
        if(!freeOrbit) {
            const Vec seed=first ? native : lastViewFront;
            orbitYaw=yawOf(seed);
            orbitPitch=pitchOf(seed);
            pitchAtGrab=orbitPitch;
        }
        if(!first) {
            orbitYaw+=std::remainder(yawOf(native)-yawOf(lastNativeFront),6.28318530718f);
            orbitPitch+=pitchOf(native)-pitchOf(lastNativeFront);
        }
        // WidescreenFix limits camera elevation relative to the stock chase
        // pitch to [-20, 60] degrees. Our vector points from eye to target.
        constexpr float rad=.01745329252f;
        orbitPitch=std::clamp(std::clamp(orbitPitch,pitchAtGrab-60*rad,pitchAtGrab+20*rad),
                              -80*rad,80*rad);
        holdTime=in.returnTimeout;
        freeOrbit=true;
    } else if(freeOrbit) {
        holdTime=std::max(0.f,holdTime-in.dt);
        if(holdTime==0 && std::hypot(in.velocity.x,in.velocity.y)>.5f) {
            const float yaw=yawOf(heading), pitch=pitchOf(heading);
            const Vec reference=returning ? previousHeading : heading;
            const float decay=std::exp(-in.returnSpeed*in.dt);
            const float yawOffset=std::remainder(orbitYaw-yawOf(reference),6.28318530718f)*decay;
            const float pitchOffset=(orbitPitch-pitchOf(reference))*decay;
            orbitYaw=yaw+yawOffset;
            orbitPitch=std::clamp(pitch+pitchOffset,-1.39626f,1.39626f);
            freeOrbit=std::max(std::abs(yawOffset),std::abs(pitchOffset))>.00436f;
            returning=freeOrbit;
        } else {
            returning=false;
        }
    }
    out.viewFront=freeOrbit ? fromAngles(orbitYaw,orbitPitch) : heading;
    previousHeading=heading;
    lastNativeFront=native;
    lastViewFront=out.viewFront;
    const float longitudinal=dot(avg,heading);
    const float eyeGain=mix(-1,mix(-.75f,-.5f,far),speedMix);
    const float eyeMin=mix(mix(-1.25f,-1.f,far),-1.f,speedMix);
    const float eyeMax=carbon ? 0.f : mix(mix(1.5f,1.f,speedMix),2.f,far);
    const float aimGain=mix(mix(-.5f,-.1f,far),mix(-.2f,-.05f,far),speedMix);
    const float aimMax=mix(2.f,.5f,speedMix);
    // The .2 brake multiplier precedes the clamp in the Carbon mover.
    // GTA supplies the brake pedal and tyre contact for its vehicle anchor.
    const float lateral=length(in.velocity-in.forward*dot(in.velocity,in.forward));
    const bool dive=carbon && in.braking && in.grounded && !in.hittingWorld &&
                    lateral < .2f*speed && in.topSpeed>0 &&
                    speed>10 && longitudinal < -.04f*in.topSpeed;
    const float eyeGoal=std::clamp(longitudinal*eyeGain*(dive?.2f:1.f),eyeMin,eyeMax)
                         +baseLag*(scale-1);
    if(first) eye.value=eye.next=baseLag*(scale-1);
    out.eye=eye.advance(eyeGoal,mix(1.25f,.75f,speedMix),in.dt);
    const float lookTime=mix(.4f,.3f,speedMix);
    out.target=target.advance(std::clamp(longitudinal*aimGain,-aimMax,aimMax),lookTime,in.dt);
    out.targetHeight=targetHeight.advance(dive ? -.35f*out.height : 0.f,lookTime,in.dt);
    const float tiltGain=mix(.004f+.001f*far,.002f+.001f*far,speedMix);
    const float tiltTime=mix(.8f,.4f,speedMix);
    out.tilt={tiltX.advance(avg.x*tiltGain,tiltTime,in.dt),
              tiltY.advance(avg.y*tiltGain,tiltTime,in.dt),0};
    return out;
}

Placement place(const Pose& native, const Vehicle& car, const Response& r, bool bumper) {
    const Vec stockFront=normalized(native.front,{0,1,0});
    const Vec front=bumper ? stockFront : normalized(r.viewFront,stockFront);
    const Vec up=bumper ? normalized(native.up,{0,0,1}) : Vec{0,0,1};
    const Vec right=normalized(cross(front,up),{1,0,0});
    Placement p;
    if(bumper) {
        p.anchor=native.position;
        p.eye=p.anchor;
        p.look=p.eye+front+up*r.targetHeight;
    } else {
        // The stock camera already accounts for the model, zoom and obstructions.
        // Pivot on its sightline so an aligned, stationary view exactly reproduces GTA.
        const float reach=std::max(.05f,dot(car.centre-native.position,stockFront));
        p.anchor=native.position+stockFront*reach;
        p.eye=p.anchor-front*reach+front*r.eye;
        p.look=p.anchor+car.forward*r.target+car.up*r.targetHeight;
    }
    p.up=up+r.tilt;
    // Apply the shaker in view space, then convert the view translation back
    // to an eye displacement. The vertical and depth components share pitch.
    constexpr float angle=6.28318530718f/65536.f;
    const float pitch=-1200.f*angle*(dot(r.shake,up)-dot(r.shake,front))+r.vibration[2];
    const float yaw=2000.f*angle*dot(r.shake,right)+r.vibration[3];
    const float distance=std::max(length(p.look-p.eye),.25f);
    const Vec f=normalized(p.look-p.eye,front);
    const Vec side=normalized(cross(f,p.up),right);
    const Vec vertical=normalized(cross(side,f),up);
    const Vec yawed=f*std::cos(yaw)+side*std::sin(yaw);
    const Vec direction=yawed*std::cos(pitch)+vertical*std::sin(pitch);
    p.eye=p.eye-r.shake-side*r.vibration[0]-vertical*r.vibration[1];
    p.look=p.eye+direction*distance;
    return p;
}
Pose orient(const Placement& p, Vec clippedEye) {
    Vec forward=normalized(p.look-clippedEye,normalized(p.look-p.eye,{0,1,0}));
    Vec right=normalized(cross(forward,p.up),normalized(cross(forward,{0,0,1}),{1,0,0}));
    return {clippedEye,forward,normalized(cross(right,forward),{0,0,1})};
}
void scaleHudVertices(const HudVertex* input, HudVertex* output, int count,
                      float centreX, float centreY, float scale) {
    for(int i=0;i<count;++i) {
        output[i]=input[i];
        output[i].x=centreX+(input[i].x-centreX)*scale;
        output[i].y=centreY+(input[i].y-centreY)*scale;
    }
}

}
