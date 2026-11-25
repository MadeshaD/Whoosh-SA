#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <cstring>
#include <vector>
#include "camera_game.hpp"
#include "game.hpp"

namespace whoosh::camera {
namespace {
constexpr uintptr_t cameraAddress=0xB6F028;
constexpr uintptr_t gameFrame=0xB7CB4C;
template<class T> void write(uintptr_t p,const T& v) {
    std::memcpy(reinterpret_cast<void*>(p),&v,sizeof(v));
}
using FindCar=uintptr_t(__cdecl*)(int,bool);
using FindPed=uintptr_t(__cdecl*)(int);
using SurfaceRoughness=unsigned(__thiscall*)(uintptr_t,unsigned);
struct State {
    uintptr_t vehicle{};
    unsigned frame{}, zoom{};
    VehicleType type{};
    Vec lastCentre{};
    std::array<float,4> compression{};
    Vec boxMin{},boxMax{};
    bool hasBox{};
    float entryTime{}, previousFov{};
    Pose previousView{};
    Vec previousCentre{};
    bool switchView{}, usedRoof{};
    Profile profile{};
    Vehicle shape{};
    Controller controller{};
    Response effect{};
};
State state;
Profile selected{};
float fovFactor=1, returnTimeout=3, returnSpeed=2;
bool enableHudShake=true;
VehicleOptions vehicles;
RoofOptions roofOptions;
struct ExitBlend {
    Pose last{};
    float fov{},elapsed{};
    unsigned frame{};
    bool valid{};
};
ExitBlend exitBlend;
using SetFov=void(__cdecl*)(float);
using Draw=void(__cdecl*)();
SetFov nextFov=reinterpret_cast<SetFov>(0x6FF410);
Draw nextHud=reinterpret_cast<Draw>(0x53E230), nextAfter=reinterpret_cast<Draw>(0x58D490), nextFonts=reinterpret_cast<Draw>(0x71A210);


uintptr_t playerVehicle(unsigned& mode, unsigned& zoom) {
    const auto c=cameraAddress;
    const unsigned active=read<uint8_t>(c+0x59);
    if(active>=3 || !read<uint8_t>(c+0x2B) || read<unsigned>(c+0xB0) ||
       read<uint8_t>(c+0x39) || read<uint8_t>(c+0x3D) || read<uint8_t>(c+0x43) ||
       read<uint8_t>(c+0xCB5))
        return 0;
    auto view=c+0x174+active*0x238;
    mode=read<uint16_t>(view+0xC);
    zoom=read<unsigned>(c+0xB4);
    if(!supported(mode,zoom)) return 0;
    if(read<uint8_t>(c+0x58)) {
        // A stock chase-to-chase transition still belongs to the same driving
        // camera. Keep the displayed pose/FOV through this transition.
        if(!zoom || !state.vehicle || !state.zoom || active>1) return 0;
        const auto other=c+0x174+(1u-active)*0x238;
        const auto otherMode=read<uint16_t>(other+0xC);
        if((otherMode!=18 && otherMode!=22) || read<uintptr_t>(other+0x21C)!=state.vehicle ||
           read<uintptr_t>(view+0x21C)!=state.vehicle) return 0;
    }
    auto car=reinterpret_cast<FindCar>(0x56E0D0)(-1,false);
    if(!car || car!=read<uintptr_t>(view+0x21C) || car!=read<uintptr_t>(c+0x958) ||
       !vehicles.allows(read<VehicleType>(car+0x594))) return 0;
    auto ped=reinterpret_cast<FindPed>(0x56E210)(-1);
    return ped && ped==read<uintptr_t>(car+0x460) ? car : 0;
}
void updateInput(uintptr_t car, uintptr_t mat, unsigned frame, unsigned zoom, float dt) {
    const Vec centre=read<Vec>(mat+0x30);
    const bool reset=state.vehicle!=car ||
                     (frame-state.frame)>1 || length(centre-state.lastCentre)>20.f;
    if(reset) {
        state={};
        state.vehicle=car;
        state.type=read<VehicleType>(car+0x594);
        state.frame=frame;
        using ModelBounds=uintptr_t(__thiscall*)(uintptr_t);
        const auto model=reinterpret_cast<ModelBounds>(0x535300)(car);
        if(model) {
            state.boxMin=read<Vec>(model);
            state.boxMax=read<Vec>(model+12);
            state.hasBox=true;
            state.shape.roof=state.boxMax.z;
            state.shape.rearExtent=-state.boxMin.y;
        }
    }
    const bool viewChanged=state.zoom!=zoom || state.usedRoof!=roofOptions.enabled || state.profile!=selected;
    if(!reset && viewChanged && exitBlend.valid) {
        state.switchView=true;
        state.previousFov=exitBlend.fov;
        state.previousView=exitBlend.last;
        state.previousCentre=state.lastCentre;
        state.entryTime=0;
    } else if(!reset && frame==state.frame) {
        return;
    }
    state.zoom=zoom;
    state.profile=selected;
    state.usedRoof=roofOptions.enabled;
    state.lastCentre=centre;
    state.frame=frame;
    state.shape.centre=centre;
    state.shape.up=read<Vec>(mat+0x20);
    state.shape.forward=read<Vec>(mat+0x10);
    Input input;
    input.profile=selected;
    input.zoom=zoom;
    input.dt=dt; input.returnTimeout=returnTimeout; input.returnSpeed=returnSpeed;
    input.velocity=read<Vec>(car+0x44)*50.f;
    input.forward=state.shape.forward;
    input.viewFront=read<Vec>(cameraAddress+0x984);

    const bool mouseHold=read<uint8_t>(0xB6EC2E) && (std::abs(read<float>(0xB73424))>0 ||
        std::abs(read<float>(0xB73428))>0);
    const bool stick=std::abs(int(read<int16_t>(0xB73458+4)))>8 ||
                     std::abs(int(read<int16_t>(0xB73458+6)))>8;
    input.manualLook=mouseHold || stick;

    const auto type=state.type;
    input.braking=usesRoadBrakes(type) && read<float>(car+0x4A0)>.01f;
    const bool carWheels=hasCarWheels(type);
    input.grounded=carWheels && read<uint8_t>(car+0x960)>0;
    auto handling=read<uintptr_t>(car+0x384);
    if(handling) input.topSpeed=read<float>(handling+0x2C+0x5C)*50.f;
    input.handbrake=cameraHandbrake(type,(read<uint8_t>(car+0x428)&0x20u)!=0);
    input.nitro=carWheels && read<unsigned>(car+0x590)==0 && (read<unsigned>(car+0x38C)&0x80000u) &&
                read<float>(car+0x8A4)<0;
    input.contact=read<uintptr_t>(car+0xDC)!=0 && read<float>(car+0xD8)>0;
    input.hittingWorld=read<uintptr_t>(car+0xDC)!=0 && read<float>(car+0xD8)>0 &&
                       std::abs(dot(read<Vec>(car+0xE0),input.forward))>.25f;
    float totalWeight=0;
    unsigned contacts=0;
    for(unsigned i=0;carWheels && i<4;++i) {
        // The completed sample is kept after ProcessControl resets its working array.
        const float compression=read<float>(car+0x7E4+4*i);
        const float change=std::abs(compression-state.compression[i]);
        state.compression[i]=compression;
        if(!input.grounded || !(compression>=0 && compression<1.f)) continue;
        const auto surface=read<uint8_t>(car+0x724+44*i+0x24);
        if(surface>=179) continue;
        const unsigned roughness=reinterpret_cast<SurfaceRoughness>(0x55E810)(0xB79538,surface);
        const Road road=wheelSurface(surface,roughness);
        totalWeight+=road.amplitude;
        input.road.frequency+=road.amplitude*road.frequency;
        ++contacts;
        if(!reset)
            input.roadPulse=std::max(input.roadPulse,change/(30.f*dt));
    }
    if(totalWeight>0 && contacts) {
        input.road.amplitude=totalWeight/contacts;
        input.road.frequency/=totalWeight;
    }
    if(!reset) state.entryTime=std::min(.55f,state.entryTime+dt);
    state.effect=state.controller.step(input);
}
bool hitGeometry(Vec from, Vec to, Vec& point) {
    ColPoint hit{};
    uintptr_t entity{};
    // Match GTA's camera sector lists. Dummy entities have no object camera policy.
    constexpr bool buildings=true, vehicles=true, peds=false, objects=true, dummies=false;
    constexpr bool seeThrough=false, checkCameraIgnore=true, shootThrough=false;
    if(!reinterpret_cast<Trace>(0x56BA00)(from,to,hit,entity,buildings,vehicles,peds,
                                          objects,dummies,seeThrough,checkCameraIgnore,shootThrough))
        return false;
    point=hit.position;
    return true;
}

void writePose(const Pose& pose,const Pose& native) {
    const auto base=cameraAddress+0x974;
    Vec right=cross(pose.front,pose.up);
    if(dot(read<Vec>(base),cross(native.front,native.up))<0) right=right*(-1);
    write(base,right); write(base+16,pose.front); write(base+32,pose.up); write(base+48,pose.position);
}
Vec localVector(Vec d,uintptr_t mat) {
    return {dot(d,read<Vec>(mat)),dot(d,state.shape.forward),dot(d,state.shape.up)};
}
float clearance(Vec eye,Vec direction,uintptr_t mat,float margin) {
    return state.hasBox ? bodyClearance(localVector(eye-state.shape.centre,mat),
        localVector(direction,mat),state.boxMin,state.boxMax,margin) : 0.f;
}
float applyCamera(float nativeFov) {
    unsigned mode{},zoom{};
    const auto car=selected!=Profile::Native && gameplay() ? playerVehicle(mode,zoom) : 0;
    const auto mat=car ? read<uintptr_t>(car+0x14) : 0;
    const float dt=read<float>(0xB7CB5C)*.02f;
    const auto base=cameraAddress+0x974;
    const Pose native{read<Vec>(base+48),read<Vec>(base+16),read<Vec>(base+32)};
    const unsigned frame=read<unsigned>(gameFrame);
    if(!(dt>0 && dt<=.15f) || selected==Profile::Native || !gameplay()) {
        state={}; exitBlend={}; return nativeFov;
    }
    if(!mat) {
        state={};
        if(!exitBlend.valid) return nativeFov;
        const unsigned active=read<uint8_t>(cameraAddress+0x59);
        const auto view=cameraAddress+0x174+active*0x238;
        const auto ped=reinterpret_cast<FindPed>(0x56E210)(-1);
        const bool playerExit=ped && active<3 && !read<unsigned>(cameraAddress+0xB0) &&
            (mode==4 || mode==18 || mode==16 || mode==22) &&
            (read<uintptr_t>(view+0x21C)==ped || !reinterpret_cast<FindCar>(0x56E0D0)(-1,false));
        if(!playerExit || frame-exitBlend.frame>1 ||
           length(native.position-exitBlend.last.position)>18.f) {
            exitBlend={}; return nativeFov;
        }
        if(exitBlend.frame!=frame) exitBlend.elapsed+=dt;
        exitBlend.frame=frame;
        const float t=unitRange(exitBlend.elapsed/.45f), w=t*t*(3-2*t);
        if(t>=1) { exitBlend={}; return nativeFov; }
        const Vec eye=limitTravel(native.position,native.position+(exitBlend.last.position-native.position)*(1-w),.2f,hitGeometry);
        const Vec front=blendBearing(exitBlend.last.front,native.front,w);
        Placement end{eye,eye,eye+front,exitBlend.last.up*(1-w)+native.up*w};
        writePose(orient(end,eye),native);
        return exitBlend.fov+(nativeFov-exitBlend.fov)*w;
    }
    const auto view=cameraAddress+0x174+read<uint8_t>(cameraAddress+0x59)*0x238;
    if(read<unsigned>(view+0x28)!=3) {
        // GTA's discrete look commands include the combined rear view.
        state={}; exitBlend={}; return nativeFov;
    }
    if(roofOptions.enabled && zoom==2) {
        // Entry or enabling the roof view on medium selects close. While cycling,
        // continue in the direction that brought the player to the medium slot.
        zoom=state.vehicle==car && state.usedRoof && state.zoom==1 ? 3u : 1u;
        write(cameraAddress+0xB4,zoom);
    }
    updateInput(car,mat,frame,zoom,dt);
    const Response& effect=state.effect;
    if((zoom && effect.lag==0) || (!zoom && effect.targetHeight==0)) return nativeFov;
    Placement desired=place(native,state.shape,effect,zoom==0,roofOptions);
    if(!std::isfinite(dot(desired.anchor,desired.anchor)) || !std::isfinite(dot(desired.eye,desired.eye)) ||
       !std::isfinite(dot(desired.look,desired.look)) || !std::isfinite(dot(native.position,native.position)))
        return nativeFov;
    const float t=state.entryTime/.55f;
    const float blend=t*t*(3-2*t);
    Pose start=native;
    float startFov=nativeFov;
    if(state.switchView && blend<1) {
        start=state.previousView;
        start.position=start.position+(state.lastCentre-state.previousCentre);
        startFov=state.previousFov;
    }
    const float fov=startFov+blend*(std::clamp((drawFov(selected,mode,zoom,nativeFov)+effect.nitroFov)*fovFactor,35.f,125.f)-startFov);
    if(blend==0) return nativeFov;
    const float reach=std::max(.25f,length(desired.look-desired.eye));
    const Vec desiredFront=(desired.look-desired.eye)*(1.f/reach);
    if(zoom && reach>.05f) {
        const Vec from=start.position-desired.anchor, to=desired.eye-desired.anchor;
        const float a=length(from), b=length(to);
        if(a>.01f && b>.01f)
            desired.eye=desired.anchor+blendBearing(from*(1/a),to*(1/b),blend)*(a+(b-a)*blend);
    } else desired.eye=start.position+(desired.eye-start.position)*blend;
    desired.look=desired.eye+blendBearing(start.front,desiredFront,blend)*reach;
    desired.up=start.up+(desired.up-start.up)*blend;
    constexpr uintptr_t ignoreEntity=0xB7CD68;
    const auto previous=read<uintptr_t>(ignoreEntity);
    write(ignoreEntity,car);
    const auto rwCam=read<uintptr_t>(0xC1703C);
    const float nearZ=rwCam ? std::clamp(read<float>(rwCam+0x80),.10f,1.f) : .3f;
    const float halfW=rwCam ? std::clamp(read<float>(rwCam+0x68),.1f,3.f) : 1.f;
    const float halfH=rwCam ? std::clamp(read<float>(rwCam+0x6c),.1f,3.f) : .75f;
    const float margin=nearZ*std::sqrt(1+halfW*halfW+halfH*halfH)+.12f;
    if(zoom) {
        const Vec radius=desired.eye-desired.anchor;
        const float distance=length(radius);
        if(distance>.01f) {
            const Vec outward=radius*(1.f/distance);
            const Vec correction=outward*clearance(desired.eye,outward,mat,margin);
            desired.eye=desired.eye+correction;
            if(desired.keepBearing) desired.look=desired.look+correction;
        }
    }
    Vec eye=limitTravel(desired.anchor,desired.eye,zoom ? .24f : .12f,hitGeometry);
    if(zoom) {
        const auto pose=orient(desired,eye);
        eye=clipNearPlane(desired.anchor,pose,nearZ,halfW,halfH,hitGeometry);
        if(length(eye-native.position)>.05f)
            eye=limitTravel(native.position,eye,.20f,hitGeometry);
    }
    if(zoom && clearance(eye,desired.look-eye,mat,margin)>0) {
        const Vec along=desired.eye-desired.anchor;
        const float requestedSetback=std::hypot(along.x,along.y);
        auto accessible=[&](Vec candidate,Vec& result) {
            Vec checked=limitTravel(desired.anchor,candidate,.20f,hitGeometry);
            checked=clipNearPlane(desired.anchor,orient(desired,checked),nearZ,halfW,halfH,hitGeometry);
            const Vec d=checked-desired.anchor;
            if(std::hypot(d.x,d.y)<.55f*requestedSetback ||
               clearance(checked,state.shape.up,mat,margin)>0) return false;
            result=checked;
            return true;
        };
        const bool found=liftLowEye(desired.anchor,desired.eye,std::max(1.f,-effect.lag*.5f),accessible,eye);
        if(!found && !desired.keepBearing) {
            // The wall limits eye travel. Keep the requested bearing even if
            // this places the eye inside the car or at the look target.
            desired.look=eye+(desired.look-desired.eye);
        }
    }
    write(ignoreEntity,previous);
    const auto pose=orient(desired,eye);
    writePose(pose,native);
    exitBlend={pose,fov,0,frame,true};
    return fov;
}
void __cdecl cameraFov(float native) {
    nextFov(applyCamera(native));
}

// The game HUD and its deferred text use RenderWare screen-space vertices.
// Transform only those two draws. Keep UVs, depth, color and caller storage intact.
using Vertex=HudVertex;
using Line=int(__cdecl*)(Vertex*,int,int,int);
using Triangle=int(__cdecl*)(Vertex*,int,int,int,int);
using Prim=int(__cdecl*)(int,Vertex*,int);
using Indexed=int(__cdecl*)(int,Vertex*,int,uint16_t*,int);
struct Dispatch { Line line; Triangle triangle; Prim primitive; Indexed indexed; };
static_assert(sizeof(Dispatch)==16);
Dispatch original{};
float screenX{},screenY{},hudScale{1};
bool inScope{}, forwarding{};
std::vector<Vertex> largeBatch;

template<class Draw> int submit(Vertex* input,int count,Draw draw) {
    if(forwarding || !input || count<=0 || count>65536) return draw(input);
    std::array<Vertex,96> local;
    Vertex* copy=local.data();
    if(static_cast<size_t>(count)>local.size()) {
        try { largeBatch.resize(count); }
        catch(...) { return draw(input); }
        copy=largeBatch.data();
    }
    scaleHudVertices(input,copy,count,screenX,screenY,hudScale);
    struct Guard { Guard(){forwarding=true;} ~Guard(){forwarding=false;} } guard;
    return draw(copy);
}
int __cdecl line(Vertex* v,int n,int a,int b) {
    return submit(v,n,[&](Vertex* q){return original.line(q,n,a,b);});
}
int __cdecl triangle(Vertex* v,int n,int a,int b,int c) {
    return submit(v,n,[&](Vertex* q){return original.triangle(q,n,a,b,c);});
}
int __cdecl primitive(int type,Vertex* v,int n) {
    return submit(v,n,[&](Vertex* q){return original.primitive(type,q,n);});
}
int __cdecl indexed(int type,Vertex* v,int n,uint16_t* indices,int size) {
    return submit(v,n,[&](Vertex* q){return original.indexed(type,q,n,indices,size);});
}
constexpr Dispatch scaled{line,triangle,primitive,indexed};
struct HudScope {
    uintptr_t slots{};
    HudScope() {
        if(!enableHudShake || inScope || selected!=Profile::Carbon || !state.vehicle ||
           state.frame!=read<unsigned>(gameFrame) || state.effect.hudScale<=1.00001f ||
           !gameplay()) return;
        auto rw=read<uintptr_t>(0xC97B24), camera=read<uintptr_t>(0xC1703C);
        auto raster=camera ? read<uintptr_t>(camera+0x60) : 0;
        if(!rw || !raster) return;
        const int w=read<int>(raster+12),h=read<int>(raster+16);
        if(w<=0 || h<=0) return;
        original=read<Dispatch>(rw+0x28);
        if(!original.line || !original.triangle || !original.primitive || !original.indexed) return;
        screenX=w*.5f; screenY=h*.5f;
        hudScale=state.effect.hudScale;
        slots=rw+0x28;
        inScope=true;
        write(slots,scaled);
    }
    ~HudScope() {
        if(slots) {
            write(slots,original);
            inScope=false;
        }
    }
};
void __cdecl hud() { HudScope scope; nextHud(); }
void __cdecl afterFade() { HudScope scope; nextAfter(); }
void __cdecl fonts() { HudScope scope; nextFonts(); }

template<class Fn> bool replaceBranch(uintptr_t addr,uint8_t opcode,Fn& previous,Fn to) {
    if(read<uint8_t>(addr)!=opcode) return false;
    auto target=addr+5+read<int32_t>(addr+1);
    if(target==reinterpret_cast<uintptr_t>(to)) return true;
    // An ASI loaded earlier may already be handling this same call site.
    // Chain its code if it retained the call/jump and the surrounding layout.
    MEMORY_BASIC_INFORMATION info{};
    if(!VirtualQuery(reinterpret_cast<void*>(target),&info,sizeof(info)) ||
       info.State!=MEM_COMMIT || (info.Protect&PAGE_GUARD) ||
       !(info.Protect&(PAGE_EXECUTE|PAGE_EXECUTE_READ|PAGE_EXECUTE_READWRITE|PAGE_EXECUTE_WRITECOPY)))
        return false;
    auto operand=reinterpret_cast<void*>(addr+1);
    DWORD protect;
    if(!VirtualProtect(operand,4,PAGE_EXECUTE_READWRITE,&protect)) return false;
    const uint32_t relative=reinterpret_cast<uintptr_t>(to)-(addr+5);
    previous=reinterpret_cast<Fn>(target);
    write(addr+1,relative);
    FlushInstructionCache(GetCurrentProcess(),operand,4);
    DWORD unused;
    VirtualProtect(operand,4,protect,&unused);
    return true;
}
}

void select(Profile p,float factor,VehicleOptions options,float timeout,float speed,bool hudShake, RoofOptions roof) {
    roofOptions=roof;
    enableHudShake=hudShake;
    returnTimeout=timeout; returnSpeed=speed;
    if(state.vehicle && !options.allows(state.type)) { state={}; exitBlend={}; }
    vehicles=options;
    fovFactor=factor;
    if(p==Profile::Native) { state={}; exitBlend={}; }
    selected=p;
}
bool active() {return state.vehicle && state.frame==read<unsigned>(gameFrame);}
void installGame() {
    if(read<uint8_t>(0x52C975)!=0x52 || read<uint32_t>(0x52C97B)!=0x6A04C483u ||
       !replaceBranch(0x52C976,0xE8,nextFov,cameraFov)) return;
    replaceBranch(0x53EB12,0xE8,nextHud,hud);
    replaceBranch(0x53EBA2,0xE8,nextAfter,afterFade);
    replaceBranch(0x53EBB1,0xE8,nextFonts,fonts);
}
extern "C" __declspec(dllexport) float __cdecl WhooshGetHudScale() {
    return enableHudShake && selected==Profile::Carbon && state.vehicle && state.frame==read<unsigned>(gameFrame) && gameplay()
        ? state.effect.hudScale : 1.f;
}
}
