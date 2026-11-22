#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <cstring>
#include <cstdio>
#include "detector.hpp"
#include "visuals.hpp"
#include "game.hpp"
#include "motion.hpp"
#include "carbonblur.hpp"

#include <string>

namespace whoosh::visuals {
namespace {
template<class R,class... Args> R call(uintptr_t addr,Args... args) {
    return reinterpret_cast<R(__cdecl*)(Args...)>(addr)(args...);
}
std::wstring texturePath;
Options settings;
bool drivingNow{},replaceNative{};
uintptr_t originalRender{};
float boostBlend{};
DWORD lastTick{};
float mwReleaseSeconds=5.f;
uintptr_t lastCar{};
void stockSpeed(float v) {if(!replaceNative)call<void>(0x7030A0,v);}

struct ScreenVertex {float x,y,z,rhw;uint32_t color;float u,v;};
static_assert(sizeof(ScreenVertex)==28);
constexpr int cols=20,rows=14;
constexpr auto meshIndices() {
    std::array<uint16_t,cols*rows*6> out{};
    int n=0;
    for(int y=0;y<rows;++y)for(int x=0;x<cols;++x) {
        int k=y*(cols+1)+x;
        for(int i:{k,k+1,k+cols+1,k+1,k+cols+2,k+cols+1})out[n++]=uint16_t(i);
    }
    return out;
}
uintptr_t effectRaster(DWORD now,const char* name);
const char* blurStage="inactive";
HRESULT blurResult{};
unsigned blurFrames{},windFrames{},lampFrames{};
void motionBlur(float strength,uintptr_t car) {
    blurResult=S_OK;
    blurStage="below speed threshold";
    if(strength<.005f)return;
    blurStage="camera and scene rasters";
    auto camera=read<uintptr_t>(0xC1703C),buffer=read<uintptr_t>(0xC402D8);
    if(!camera || !buffer)return;
    auto source=read<uintptr_t>(camera+0x60);
    if(!source || source==buffer)return;
    int w=read<int>(source+12),h=read<int>(source+16);
    int bw=read<int>(buffer+12),bh=read<int>(buffer+16);
    if(w<=0 || h<=0 || bw<w || bh<h)return;
    auto rw=read<uintptr_t>(0xC97B24);
    if(!rw)return;
    auto set=read<uintptr_t>(rw+0x20),get=read<uintptr_t>(rw+0x24);
    uintptr_t oldTexture{};
    blurStage="read previous texture";
    if(!call<int>(get,1,&oldTexture))return;
    auto m=read<uintptr_t>(car+0x14);
    if(!m)return;
    const auto view=camera+0x20;
    Projection proj{read<Vec>(view),read<Vec>(view+16),read<Vec>(view+32),read<Vec>(view+48)};
    Vec velocity=read<Vec>(car+0x44);
    float speed=length(velocity);
    if(speed<.015f)return;
    Vec motion=proj.vector(velocity*(1.f/speed));
    Vec centre=proj.point(read<Vec>(m+48));
    // Protect the vehicle with a feathered screen mask from its projected collision bounds.
    UV mid{},radius{};
    auto col=reinterpret_cast<uintptr_t(__thiscall*)(uintptr_t)>(0x535300)(car);
    if(col && centre.z>1.f) {
        Vec lo=read<Vec>(col),hi=read<Vec>(col+12);
        UV mn{2,2},mx{-1,-1};
        bool visible=true;
        for(unsigned k=0;k<8;++k) {
            Vec q=centre+proj.vector(read<Vec>(m)*((k&1)?hi.x:lo.x)
                    +read<Vec>(m+16)*((k&2)?hi.y:lo.y)+read<Vec>(m+32)*((k&4)?hi.z:lo.z));
            if(q.z<.5f){visible=false;break;}
            float x=q.x/q.z,y=q.y/q.z;
            mn={std::min(mn.x,x),std::min(mn.y,y)};
            mx={std::max(mx.x,x),std::max(mx.y,y)};
        }
        if(visible){mid={(mn.x+mx.x)*.5f,(mn.y+mx.y)*.5f};radius={(mx.x-mn.x)*.55f,(mx.y-mn.y)*.55f};}
    }
    // Follow the game's camera update sequence for copying the current scene.
    blurStage="copy current scene";
    call<void*>(0x7EE180,camera);
    bool copied=false;
    if(call<void*>(0x7FB060,buffer)) {
        copied=call<void*>(0x7FAF50,source,0,0)!=nullptr;
        call<void*>(0x7FB110);
    }
    if(!call<int>(0x619450,camera) || !copied)return;
    const bool useCarbon=settings.blur==2;
    const float normalizedSpeed=std::clamp((speed*50.f-10.f)/120.f,0.f,.5f);
    const float exposure=useCarbon ? .024f*normalizedSpeed*(1+boostBlend)*settings.strength
                                   : .014f*strength;
    const auto filtered=carbon::filter(buffer,w,h,bw,bh,motion,exposure,useCarbon,blurStage,blurResult);
    if(!filtered)return;
    call<void>(0x700CC0);
    call<void>(0x700D70);
    call<int>(set,9,2); // bilinear
    call<int>(set,1,buffer);
    uintptr_t oldAlpha{};
    bool restoreAlpha=call<int>(get,29,&oldAlpha)!=0;
    if(restoreAlpha)call<int>(set,29,8);
    auto draw=read<uintptr_t>(rw+0x34);
    static constexpr auto indices=meshIndices();
    std::array<ScreenVertex,(cols+1)*(rows+1)> vertices{};
    for(int y=0;y<=rows;++y)for(int x=0;x<=cols;++x) {
        UV p{float(x)/cols,float(y)/rows};
        float mask=1;
        if(radius.x>.001f && radius.y>.001f) {
            float dx=(p.x-mid.x)/radius.x,dy=(p.y-mid.y)/radius.y;
            mask=smooth((std::sqrt(dx*dx+dy*dy)-.72f)/.55f);
        }
        vertices[y*(cols+1)+x]={p.x*w-.5f,p.y*h-.5f,0,1,
            (uint32_t(255*mask)<<24)|0xffffff,0,0};
    }
    // Composite one filtered scene under the HUD, feathering the protected vehicle.
    call<int>(set,1,filtered);
    const float mix=useCarbon ? unitRange(4.f*normalizedSpeed*settings.strength) : unitRange(3.f*strength);
    for(auto& v:vertices) {
        v.u=(v.x+.5f)/w;
        v.v=(v.y+.5f)/h;
        v.color=(unsigned((v.color>>24)*mix)<<24)|0xffffff;
    }
    if(call<int>(draw,3,vertices.data(),int(vertices.size()),indices.data(),int(indices.size()))) {
        ++blurFrames;
        blurStage="composite submitted";
    } else blurStage="composite rejected";
    call<int>(set,1,buffer);
    if(useCarbon) {
        // Reintroduce unblurred scenery at increasing Z thresholds using the live
        // scene depth buffer. Never write depth or assume a sampleable D3D9 depth texture.
        float nearZ=read<float>(camera+0x80),farZ=read<float>(camera+0x84);
        if(nearZ>0 && farZ>nearZ) {
            call<int>(set,6,1);
            for(int band=0;band<12;++band) {
                float metres=10.f+(band+.5f)*(340.f/12);
                if(metres>=farZ)break;
                float z=farZ/(farZ-nearZ)*(1-nearZ/metres);
                uint32_t color=(uint32_t(std::lround(255.f*.30f/(12.f-.30f*band)))<<24)|0xffffff;
                ScreenVertex q[4]={{-.5f,-.5f,z,1,color,0,0},{w-.5f,-.5f,z,1,color,float(w)/bw,0},
                    {-.5f,h-.5f,z,1,color,0,float(h)/bh},{w-.5f,h-.5f,z,1,color,float(w)/bw,float(h)/bh}};
                call<int>(read<uintptr_t>(rw+0x30),4,q,4);
            }
        }
    }
    if(useCarbon) {
        auto vignette=effectRaster(lastTick,"UVESVIGNETTE_B");
        if(vignette) {
            call<int>(set,6,0);
            call<int>(set,1,vignette);
            uint32_t tint=(unsigned(255.f*unitRange(4.f*normalizedSpeed*settings.strength))<<24)|0xffffff;
            ScreenVertex q[4]={{-.5f,-.5f,0,1,tint,0,0},{w-.5f,-.5f,0,1,tint,1,0},
                {-.5f,h-.5f,0,1,tint,0,1},{w-.5f,h-.5f,0,1,tint,1,1}};
            call<int>(read<uintptr_t>(rw+0x30),4,q,4);
        }
    }
    if(restoreAlpha)call<int>(set,29,oldAlpha);
    call<int>(set,1,oldTexture);
    call<void>(0x700E00);
}

struct Lamp {Vec centre{},wake{};float radius{};DWORD sampleTime{};};
std::array<Lamp,2> tracks;
// XenonEffects contrail -> fuelcell_emitter 1875e3d5 -> emitteruv 9ccc9411.
struct WindParticle {
    Vec start{}, velocity{};
    float ageAtBirth{}, lengthTime{};
    uint32_t color{};
    DWORD born{};
};
constexpr unsigned windCount=15;
constexpr float windLife=.25f*(1.f-.25f);
std::array<WindParticle,windCount*7> windParticles{};
size_t nextWind{};
Vec lastWindOrigin{};
DWORD windTick{};
unsigned windPhase{};
bool windActive{};
uint32_t windSeed=0xdeadbeef;
float windRandom(float range) {
    const uint32_t sample=windSeed%0x7fffffff;
    const uint32_t next=windSeed^0x1d872b41;
    const uint32_t temp=next^(next>>5);
    windSeed=temp^next^(temp<<27);
    return range*float(sample)*4.656613e-10f;
}
Vec windPosition(Vec start,Vec velocity,float age) {
    Vec point=start+velocity*age;
    point.z-=2.f*age*age;
    return point;
}
void clearTrails() {
    tracks={};windParticles={};windActive=false;
}
void updateWind(Vec origin,float speed,Vec velocity,DWORD now) {
    if(windActive && length(origin-lastWindOrigin)>20.f)clearTrails();
    lastWindOrigin=origin;
    if(!settings.wind || settings.windOpacity<=0 || speed<settings.windStart) {
        if(!settings.wind || settings.windOpacity<=0)windParticles={};
        windActive=false;
        return;
    }
    const DWORD elapsed=windActive?now-windTick:0;
    windTick=now;
    windPhase=windActive?windPhase+elapsed*30:1000;
    windActive=true;
    if(windPhase<1000)return;
    windPhase%=1000;
    const float intensity=.10f+.65f*unitRange((speed-settings.windStart)/settings.windStart);
    const uint32_t color=(uint32_t(42.f*1.15f*intensity*settings.windOpacity)<<24)|0xa3ffd8u;
    Vec drift={-.3f*velocity.x,-.3f*velocity.y,-.1f*velocity.z-1.f};
    // AddXenonEffect copies the identity basis and only the car translation.
    for(unsigned i=0;i<windCount;++i) {
        float lengthTime=float(int(200.f+windRandom(50.f)))/2048.f;
        for(int j=0;j<4;++j)windRandom(0.f); // velocity XYZ and gravity deltas
        Vec offset={windRandom(1.5f)-.75f,windRandom(2.f)-1.f,windRandom(.75f)-.375f};
        float age=float(elapsed)*.001f*i/windCount;
        windParticles[nextWind++%windParticles.size()]={
            windPosition(origin+offset,drift,age),drift,age,lengthTime,color,now};
    }
}
void updateTrails(uintptr_t car,float speed,bool nitro,DWORD now) {
    auto matrix=read<uintptr_t>(car+0x14);
    if(!matrix){clearTrails();return;}
    Vec origin=read<Vec>(matrix+48);
    updateWind(origin,speed,read<Vec>(car+0x44)*50.f,now);
    auto index=read<unsigned short>(car+0x22);
    auto model=index<20000?read<uintptr_t>(0xA9B0C8+4*index):0;
    auto dummies=model?read<uintptr_t>(model+0x5c):0;
    if(!dummies){tracks={};return;}
    Vec marker=read<Vec>(dummies+12); // VEHICLE_DUMMY_LIGHT_REAR_MAIN
    if(length(marker)<.15f || length(marker)>15.f){tracks={};return;}
    Vec x=read<Vec>(matrix),y=read<Vec>(matrix+16),z=read<Vec>(matrix+32);
    Vec rear=origin+y*marker.y+z*marker.z;
    const BYTE overrideLights=(read<BYTE>(car+0x4a8)>>3)&3;
    bool lightsOn=overrideLights==2 || (overrideLights!=1 && (read<BYTE>(car+0x428)&0x40));
    BYTE active=read<BYTE>(car+0x584);
    for(unsigned side=0;side<tracks.size();++side) {
        auto& t=tracks[side];
        if(!settings.lights || !nitro || !lightsOn || !(active&(side?8:4))) {t={};continue;}
        Vec centre=rear+x*(marker.x*(side?-1.f:1.f));
        // GTA provides positions, but no reliable dimensions for an individual lens.
        const Vec point=centre-y*.055f;
        const auto elapsed=now-t.sampleTime;
        if(t.radius && !elapsed) continue;
        Vec wake=read<Vec>(car+0x44)*(-50.f/60.f);
        if(t.radius && elapsed<=100 && length(t.centre-point)<10.f)
            wake=(t.centre-point)*(1000.f/(60.f*elapsed));
        t={point,wake,.18f,now};

    }
}
struct Vertex {Vec position,normal;uint32_t color;float u,v;};
static_assert(sizeof(Vertex)==36);
void drawLamps(uintptr_t set) {
    int slot=call<int>(0x731850,"fx");
    if(slot<0)return;
    auto dict=call<uintptr_t>(0x408340,slot);
    auto tex=dict?call<uintptr_t>(0x7F39F0,dict,"sphere"):0;
    auto raster=tex?read<uintptr_t>(tex):0;
    auto camera=read<uintptr_t>(0xC1703C);
    auto frame=camera?read<uintptr_t>(camera+4):0;
    if(!raster || !frame)return;
    auto m=call<uintptr_t>(0x7F0990,frame);
    if(!m)return;
    Vec right=read<Vec>(m),up=read<Vec>(m+16);
    std::array<Vertex,2*10*6> quads{};
    unsigned n=0;
    for(const auto& lamp:tracks) {
        if(!lamp.radius)continue;
        // Overlap soft, camera-facing discs along a short wake. The far end goes to zero.
        for(int j=9;j>=0;--j) {
            float t=j/9.f,fade=(1.f-t)*(1.f-t);
            Vec c=lamp.centre+lamp.wake*t;
            Vec w=right*(lamp.radius*(1.5f+.55f*t)),h=up*(lamp.radius*(1.f+.65f*t));
            uint32_t color=(uint32_t(j==0?175:65*fade)<<24)|(j==0?0xff6632u:0xff1610u);
            Vertex a{c-w+h,{},color,0,0},b{c+w+h,{},color,1,0};
            Vertex d{c-w-h,{},color,0,1},e{c+w-h,{},color,1,1};
            for(auto v:{a,b,d,b,e,d})quads[n++]=v;
        }
    }
    call<int>(set,1,raster);
    call<int>(set,9,2);
    call<int>(set,11,2); // additive light
    if(n && call<void*>(0x7EF450,quads.data(),n,nullptr,25u)) {
        call<int>(0x7EF6B0,3);call<int>(0x7EF520);
    }
}

void drawTracks(DWORD now) {
    // MAIN.dds cell (128,192)-(160,224), with its original filtering border.
    // XSpriteManager truncates UV*255 to bytes before converting back to float.
    constexpr float u0=127.f/255,u1=159.f/255;
    constexpr float v0=191.f/255,v1=223.f/255;
    static std::array<Vertex,windParticles.size()*6> batch;
    unsigned used=0;
    for(auto& w:windParticles) {
        if(!w.color)continue;
        float age=w.ageAtBirth+float(now-w.born)*.001f;
        if(age>windLife){w.color=0;continue;}
        Vec a=windPosition(w.start,w.velocity,age);
        Vec b=windPosition(w.start,w.velocity,age+w.lengthTime);
        constexpr Vec height={0,0,255.f/2048};
        Vertex p{a,{},w.color,u0,v0},q{a+height,{},w.color,u1,v0};
        Vertex r{b+height,{},w.color,u1,v1},t{b,{},w.color,u0,v1};
        for(auto v:{p,q,r,p,r,t})batch[used++]=v;
    }
    auto raster=used?effectRaster(now,"WINDTRAIL"):0;
    if(!raster)used=0;
    if(!used && !tracks[0].radius && !tracks[1].radius)return;
    auto rw=read<uintptr_t>(0xC97B24);
    if(!rw)return;
    auto get=read<uintptr_t>(rw+0x24),set=read<uintptr_t>(rw+0x20);
    constexpr unsigned states[]={1,6,8,10,11,12,20,29,9};
    constexpr unsigned values[]={0,1,0,5,6,1,1,8,2};
    uintptr_t saved[9]{};
    for(unsigned i=0;i<9;++i)if(!call<int>(get,states[i],&saved[i]))return;
    for(unsigned i=0;i<9;++i)call<int>(set,states[i],values[i]);
    if(used){++windFrames;call<int>(set,1,raster);call<int>(set,11,2);}
    if(used && call<void*>(0x7EF450,batch.data(),used,nullptr,25u)) {
        call<int>(0x7EF6B0,3);
        call<int>(0x7EF520);
    }
    if(tracks[0].radius || tracks[1].radius){++lampFrames;drawLamps(set);}
    for(unsigned i=0;i<9;++i)call<int>(set,states[i],saved[i]);
}
// Collect final world coordinates and colors at the two active 2DFX emitter calls.
struct Light {Vec pos;float size,range,pull;unsigned rgb,alpha;bool below;};
std::array<Light,512> lights;
unsigned lightCount{},flareFrames{};
uintptr_t previousCorona[2]{};
template<unsigned kind>
void __cdecl observeLight(uint32_t id,uintptr_t entity,unsigned r,unsigned g,unsigned b,unsigned alpha,
    const Vec* pos,float size,float range,uintptr_t textureOrType,unsigned flare,unsigned reflection,
    unsigned los,unsigned trail,float angle,unsigned neon,float pull,unsigned full,float fade,unsigned below,unsigned core) {
    if(drivingNow && settings.flares && pos && !entity) {
        unsigned rgb=((r&255)<<16)|((g&255)<<8)|(b&255);
        if(lightCount<lights.size() && rgb && (alpha&255) && size>0 && range>0)
            lights[lightCount++]={*pos,size,range,pull,rgb,alpha&255,(below&255)!=0};
    }
    call<void>(previousCorona[kind],id,entity,r,g,b,alpha,pos,size,range,textureOrType,flare,
        reflection,los,trail,angle,neon,pull,full,fade,below,core);
}
// Keep the dictionary in a private, referenced GTA slot across save loads.
uintptr_t effectRaster(DWORD now,const char* name) {
    static int slot=-1;

    static DWORD failedAt{};
    static bool failed{};
    if(failed && now-failedAt<2000)return 0;
    if(slot<0) {
        HANDLE file=CreateFileW(texturePath.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(file==INVALID_HANDLE_VALUE){failed=true;failedAt=now;return 0;}
        LARGE_INTEGER size{};
        HANDLE mapping=nullptr;
        if(GetFileSizeEx(file,&size) && size.QuadPart>0 && size.HighPart==0)
            mapping=CreateFileMappingW(file,nullptr,PAGE_READONLY,0,0,nullptr);
        CloseHandle(file);
        auto bytes=mapping?MapViewOfFile(mapping,FILE_MAP_READ,0,0,0):nullptr;
        if(mapping)CloseHandle(mapping);
        struct Memory {const void* bytes;DWORD size;};
        Memory mem{bytes,size.LowPart};
        auto stream=bytes?call<uintptr_t>(0x7ECEF0,3,1,&mem):0;
        if(stream) {
            int fresh=call<int>(0x731C80,"whoosh_streaks");
            if(fresh>=0) {
                if(call<bool>(0x731DD0,fresh,stream)) {
                    call<void*>(0x731A00,fresh);
                    slot=fresh;
                } else call<void>(0x731CD0,fresh);
            }
            call<int>(0x7ECE20,stream,nullptr);
        }
        if(bytes)UnmapViewOfFile(bytes);
    }
    auto dict=slot>=0?call<uintptr_t>(0x408340,slot):0;
    auto texture=dict?call<uintptr_t>(0x7F39F0,dict,name):0;
    auto raster=texture?read<uintptr_t>(texture):0;
    failed=!dict;
    if(failed)failedAt=now;

    return raster;
}
// Project through the same GTA function and with the same near pull as the native light sprite.
void streetFlares(uintptr_t car,DWORD now) {
    if(!settings.flares)return;
    auto camera=read<uintptr_t>(0xC1703C);
    auto frame=camera?read<uintptr_t>(camera+4):0;
    auto matrix=frame?call<uintptr_t>(0x7F0990,frame):0;
    auto target=camera?read<uintptr_t>(camera+0x60):0;
    if(!matrix || !target)return;
    auto raster=effectRaster(now,"STREAKFLARES_I");
    if(!raster)return;
    Vec velocity=read<Vec>(car+0x44);
    float travel=length(velocity);
    if(!std::isfinite(travel))return;
    float alignment=travel>.001f?1.f-unitRange(std::abs(dot(velocity,read<Vec>(matrix)))/travel):0;
    float roadSpeed=travel*50.f;
    float onset=smooth((roadSpeed-12.5f)/15.f);
    float speed=(roadSpeed+20.f*boostBlend)*alignment;
    float brightness=onset*unitRange(speed/95.f);
    if(brightness<.02f)return;
    float back=onset*(3.f+45.f*unitRange(speed/140.f));
    float width=float(read<int>(target+12)),height=float(read<int>(target+16));
    float nearZ=read<float>(camera+0x80),farZ=read<float>(camera+0x84);
    if(width<=0 || height<=0 || nearZ<=0 || farZ<=nearZ)return;
    Vec eye=read<Vec>(matrix+48);
    float row=float((uint64_t(now)*15/1000)%6)*.125f;
    static std::array<ScreenVertex,lights.size()*6> batch;
    unsigned n=0;
    for(unsigned i=0;i<lightCount;++i) {
        const Light& light=lights[i];
        Vec pos=light.pos,ray=pos-eye;
        float distance=length(ray);
        float distanceGain=1.f-smooth((distance-18.f)/37.f);
        if(distanceGain<=0 || distance<1 || (light.below && ray.z<0) || distance>=light.range)continue;
        pos=pos-ray*(std::clamp(light.pull,0.f,distance*.5f)/distance);
        Vec screen{};
        float sx{},sy{};
        if(!call<bool>(0x70CE30,&pos,&screen,&sx,&sy,true,true))continue;
        float range=light.range;
        if(screen.z>=range || range<=0)continue;
        float dx=screen.x-width*.5f,dy=screen.y-height*.5f;
        float radial=std::hypot(dx,dy);
        if(radial<1.f)continue;
        float extent=distanceGain*radial*back/(screen.z+back);
        float halfWidth=std::clamp(sy*light.size*.22f,height/720.f,height*.025f);
        float endU=std::clamp(10.f*extent*std::hypot(dx/(width*.5f),dy/(height*.5f))/radial,.5f/128,127.5f/128);
        unsigned rgb=light.rgb;
        unsigned alpha=unsigned(light.alpha*distanceGain*brightness*unitRange(2.f*(1.f-screen.z/range)));
        if(!rgb || !alpha || extent<1.f)continue;
        float z=farZ/(farZ-nearZ)*(1.f-nearZ/screen.z);
        auto vertex=[&](float t,int edge) {
            return ScreenVertex{screen.x-(dx*extent*t+dy*halfWidth*edge)/radial,
                screen.y+(-dy*extent*t+dx*halfWidth*edge)/radial,z,1.f/screen.z,
                (alpha<<24)|rgb,t?endU:.5f/128,row+(edge>0?.5f:15.5f)/128};
        };
        for(auto point:{vertex(0,1),vertex(0,-1),vertex(1,-1),vertex(0,1),vertex(1,-1),vertex(1,1)})
            batch[n++]=point;
    }
    auto rw=read<uintptr_t>(0xC97B24);
    bool drawn=false;
    if(n && rw) {
        auto set=read<uintptr_t>(rw+0x20),get=read<uintptr_t>(rw+0x24);
        uintptr_t previous{},oldAlpha{};
        if(call<int>(get,1,&previous)) {
            bool hasAlpha=call<int>(get,29,&oldAlpha)!=0;
            call<void>(0x700CC0);
            call<void>(0x700D70);
            call<int>(set,1,raster);
            call<int>(set,6,1);
            call<int>(set,9,2);
            call<int>(set,11,2);
            if(hasAlpha)call<int>(set,29,8);
            drawn=call<int>(read<uintptr_t>(rw+0x30),3,batch.data(),n)!=0;
            call<int>(set,1,previous);
            if(hasAlpha)call<int>(set,29,oldAlpha);
            call<void>(0x700E00);
        }
    }
    flareFrames+=drawn;
}

void render() {
    uintptr_t car=drivingNow?call<uintptr_t>(0x56E0D0,-1,false):0;
    float strength=0;
    replaceNative=car && settings.blur && settings.strength>0;
    if(car) {
        float speed=length(read<Vec>(car+0x44))*50.f;
        bool nitro=read<unsigned>(car+0x590)==0 && (read<unsigned>(car+0x38c)&0x80000u) && read<float>(car+0x8a4)<0;
        DWORD now=read<DWORD>(0xB7CB84);
        if(lastCar!=car || now-lastTick>250){boostBlend=0;mwReleaseSeconds=5.f;clearTrails();}
        mwReleaseSeconds=nitro?0.f:std::min(5.f,mwReleaseSeconds+float(now-lastTick)*.001f);
        boostBlend=nitro?1.f:std::max(0.f,boostBlend-.005f*float(now-lastTick));
        lastTick=now;
        if(std::isfinite(speed)) {
            strength=settings.strength*blurDrive(settings.blur==2,speed,boostBlend);
            if(settings.blur==1) {
                float pulse=mwNitro(mwReleaseSeconds);
                strength=settings.strength*std::max(blurDrive(false,speed,0),pulse*unitRange(speed/18.f));
            }
            updateTrails(car,speed,nitro,now);
            drawTracks(now);
        }
    }
    lastCar=car;
    call<void>(originalRender);
    if(replaceNative)motionBlur(strength,car);
    else blurStage="inactive";
    if(car)streetFlares(car,lastTick);
    lightCount=0;
    replaceNative=false;
}
bool redirect(uintptr_t site,uintptr_t dest) {
    DWORD old;
    if(!VirtualProtect(reinterpret_cast<void*>(site+1),4,PAGE_EXECUTE_READWRITE,&old))return false;
    uint32_t disp=uint32_t(dest-site-5);
    std::memcpy(reinterpret_cast<void*>(site+1),&disp,4);
    FlushInstructionCache(GetCurrentProcess(),reinterpret_cast<void*>(site),5);
    VirtualProtect(reinterpret_cast<void*>(site+1),4,old,&old);
    return true;
}
template<unsigned kind>
void hookEmitter(uintptr_t site) {
    // Preserve any existing cdecl wrapper at this call site.
    if(read<BYTE>(site)!=0xe8)return;
    previousCorona[kind]=site+5+read<int32_t>(site+1);
    if(!redirect(site,reinterpret_cast<uintptr_t>(&observeLight<kind>)))previousCorona[kind]=0;
}

}
void setTexturePath(const wchar_t* path){texturePath=path;}
void report(FILE* file) {
    std::fprintf(file,"BlurStage=%s HRESULT=%08lX BlurFrames=%u WindTrailBatches=%u NitroLightBatches=%u FlareFrames=%u\n",
                 blurStage,static_cast<unsigned long>(blurResult),blurFrames,windFrames,lampFrames,flareFrames);
}
void configure(Options value){settings=value;}
void driving(bool active){drivingNow=active;}
void install() {
    constexpr uintptr_t effectCall=0x53ead3,speedCall=0x704e8a;
    if(read<BYTE>(effectCall)!=0xe8 || read<unsigned>(effectCall+5)!=0xb6f0b8a1u)return;
    originalRender=effectCall+5+read<int32_t>(effectCall+1);
    if(!redirect(effectCall,reinterpret_cast<uintptr_t>(render)))return;
    if(read<uint32_t>(0x6fd033)==0xeb54c483)hookEmitter<0>(0x6fd02e);
    if(read<uint32_t>(0x49dd51)==0x8314468b)hookEmitter<1>(0x49dd4c);
    if(read<BYTE>(speedCall)==0xe8 && speedCall+5+read<int32_t>(speedCall+1)==0x7030a0)
        redirect(speedCall,reinterpret_cast<uintptr_t>(stockSpeed));
}
}
