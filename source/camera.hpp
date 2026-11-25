#pragma once
#include "detector.hpp"

namespace whoosh::camera {
enum class Profile { Native, MostWanted, Carbon };
// CVehicle::m_nVehicleSubClass in the supported SA executable.
enum class VehicleType : unsigned {
    Automobile, MonsterTruck, Quad, Helicopter, Plane, Boat, Train,
    FakeHelicopter, FakePlane, Motorcycle, Bicycle, Trailer
};
struct VehicleOptions {
    bool boats{}, motorcycles{}, planes{}, helicopters{}, trains{};
    bool allows(VehicleType type) const {
        switch(type) {
        case VehicleType::Automobile: case VehicleType::MonsterTruck: case VehicleType::Quad: return true;
        case VehicleType::Motorcycle: case VehicleType::Bicycle: return motorcycles;
        case VehicleType::Helicopter: case VehicleType::FakeHelicopter: return helicopters;
        case VehicleType::Plane: case VehicleType::FakePlane: return planes;
        case VehicleType::Boat: return boats;
        case VehicleType::Train: return trains;
        default: return false;
        }
    }
};
constexpr bool hasCarWheels(VehicleType type) {
    return type==VehicleType::Automobile || type==VehicleType::MonsterTruck || type==VehicleType::Quad;
}
constexpr bool usesRoadBrakes(VehicleType type) {
    return hasCarWheels(type) || type==VehicleType::Motorcycle || type==VehicleType::Bicycle;
}
constexpr bool cameraHandbrake(VehicleType type,bool applied) {
    return applied && usesRoadBrakes(type) && type!=VehicleType::MonsterTruck;
}
bool supported(unsigned mode, unsigned zoom);
float drawFov(Profile profile, unsigned mode, unsigned zoom, float nativeFov);

struct Road { float amplitude{}, frequency{}; };
Road wheelSurface(unsigned surface, unsigned roughness);

struct Input {
    Profile profile{};
    unsigned zoom{};
    Vec velocity{}, forward{0,1,0}, viewFront{0,1,0};
    float topSpeed{}, dt{}, roadPulse{}, returnTimeout{3}, returnSpeed{2};
    Road road{};
    bool braking{}, grounded{}, nitro{}, handbrake{}, hittingWorld{}, contact{}, manualLook{};
};
struct Response {
    float nitroFov{}, lag{}, height{}, eye{}, target{}, targetHeight{}, hudScale{1};
    Vec tilt{}, shake{}, viewFront{};
    std::array<float,4> vibration{}; // view translation X/Y in m, pitch/yaw in radians
};
class Controller {
    struct Cubic {
        float value{}, slope{}, next{};
        float advance(float goal, float duration, float dt);
    };
    Cubic nitroFov, eye, target, targetHeight, tiltX, tiltY, headingX, headingY, headingZ;
    Profile lastProfile{};
    unsigned lastZoom{};
    bool started{}, freeOrbit{};
    float orbitYaw{}, orbitPitch{}, pitchAtGrab{}, holdTime{};
    Vec lastNativeFront{}, lastViewFront{}, previousHeading{};
    bool returning{};
    Vec lastVelocity{}, shakeDirection{};
    std::array<Vec,5> samples{};
    unsigned count{}, cursor{};
    float phase{1}, duration{1};
    double roadClock{};
public:
    Response step(const Input& in);
};

struct Pose { Vec position{}, front{0,1,0}, up{0,0,1}; };
Vec blendBearing(Vec from,Vec to,float weight);
struct Vehicle { Vec centre{}, up{0,0,1}, forward{0,1,0}; float rearExtent{}, roof{}; };
// If the requested eye lies inside the padded vehicle box, slide it out
// along the same orbit radius. Inputs are in the vehicle's local axes.
inline float bodyClearance(Vec local,Vec direction,Vec lo,Vec hi,float margin) {
    float exit=1e6f;
    const auto slab=[&](float p,float d,float a,float b) {
        if(p<a-margin || p>b+margin) return false;
        if(std::abs(d)>1.e-5f)
            exit=std::min(exit,((d>0 ? b+margin : a-margin)-p)/d);
        return true;
    };
    if(!slab(local.x,direction.x,lo.x,hi.x) || !slab(local.y,direction.y,lo.y,hi.y) ||
       !slab(local.z,direction.z,lo.z,hi.z) || exit==1e6f) return 0;
    return exit+.02f;
}
// Lift a blocked low chase view on its current bearing, retaining its
// horizontal setback so a ground contact cannot produce a roof camera.
template<class Accept> bool liftLowEye(Vec pivot,Vec requested,float minimumSetback,Accept accept,Vec& result) {
    const Vec offset=requested-pivot;
    const float horizontal=std::hypot(offset.x,offset.y);
    if(horizontal<.05f) return false;
    const float setback=std::max(horizontal,minimumSetback);
    Vec candidate=pivot+Vec{offset.x,offset.y,0}*(setback/horizontal);
    const float first=std::max(offset.z,-.5f*setback);
    for(unsigned i=0;i<=12;++i) {
        candidate.z=pivot.z+first+.10f*setback*i;
        if(accept(candidate,result)) return true;
    }
    return false;
}
struct Placement { Vec anchor{}, eye{}, look{}, up{}; bool keepBearing{}; };
struct RoofOptions { bool enabled{}; float height{}, distance{}, pitchDegrees{}; };
Placement place(const Pose& native, const Vehicle& car, const Response& response, bool bumper, RoofOptions roof={});
Pose orient(const Placement& p, Vec clippedEye);
Vec cross(Vec a, Vec b);
template<class Cast> Vec limitTravel(Vec from, Vec to, float clearance, Cast cast) {
    const Vec delta=to-from;
    const float distance=length(delta);
    if(distance<.001f) return to;
    const Vec along=delta*(1.f/distance);
    Vec hit{};
    if(!cast(from,to+along*clearance,hit)) return to;
    const float stop=std::clamp(dot(hit-from,along)-clearance,0.f,distance);
    return from+along*stop;
}
// Sweep the near-plane corners along the boom. A parallel sweep lets us
// subtract clearance in world units even when the plane grazes a side wall.
template<class Cast> Vec clipNearPlane(Vec pivot,const Pose& view,float nearZ,
                                      float halfW,float halfH,Cast cast) {
    const Vec delta=view.position-pivot;
    const float distance=length(delta);
    if(distance<.01f) return view.position;
    const Vec along=delta*(1.f/distance);
    const Vec right=cross(view.front,view.up);
    float stop=distance;
    for(float x:{-1.f,1.f}) for(float y:{-1.f,1.f}) {
        const Vec corner=(view.front+right*(x*halfW)+view.up*(y*halfH))*nearZ;
        const Vec start=pivot+corner;
        Vec hit{};
        if(cast(start,view.position+corner+along*.12f,hit))
            stop=std::min(stop,std::max(0.f,dot(hit-start,along)-.12f));
    }
    return pivot+along*stop;
}
struct HudVertex { float x,y,z,rhw; uint32_t color; float u,v; };
static_assert(sizeof(HudVertex)==28);
void scaleHudVertices(const HudVertex* input, HudVertex* output, int count,
                      float centreX, float centreY, float scale);

}
