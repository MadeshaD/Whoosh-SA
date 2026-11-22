#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "audio.hpp"
#include "camera_game.hpp"
#include "visuals.hpp"
#include "geometry.hpp"
#include "game.hpp"
#include <cstring>
#include <cwchar>

using namespace whoosh;
namespace {
static_assert(sizeof(void*)==4,"Build for Win32 / x86.");
using Nearby=void(__cdecl*)(const Vec&,float,bool,short*,short,uintptr_t*,bool,bool,bool,bool,bool);
using GetCol=uintptr_t(__thiscall*)(uintptr_t);
using FindVehicle=uintptr_t(__cdecl*)(int,bool);
using FindPed=uintptr_t(__cdecl*)(int);
using GameService=void(__thiscall*)(void*);
GameService previousService{};
HMODULE module{};
Detector detector;
bool initialized{},enabled{},wasDriving{};
uintptr_t lastVehicle{};
DWORD lastGameTime{};
float settingGain=.8f;
std::wstring directory,ini;
bool keyHeld=false,logging=false;
unsigned passEvents{};
DWORD lastReport{};
void reportStatus() {
    if(!logging)return;
    DWORD now=GetTickCount();
    if(now-lastReport<10000)return;
    lastReport=now;
    if(FILE* file=_wfopen((directory+L"\\Whoosh.SA.log").c_str(),L"ab")) {
        std::fprintf(file,"UptimeMs=%lu Driving=%d CameraActive=%d PassEvents=%u\n",now,wasDriving,camera::active(),passEvents);
        audio::report(file);
        visuals::report(file);
        std::fclose(file);
    }
}
int reloadKey=VK_F9;

Vec position(uintptr_t entity) {
    auto m=read<uintptr_t>(entity+0x14);
    return m?read<Vec>(m+0x30):read<Vec>(entity+4);
}
bool describe(uintptr_t entity,Vec along,Body& b) {
    unsigned flags=read<unsigned>(entity+0x1C);
    if(!(flags&1) && (!(flags&0x80) || !read<uintptr_t>(entity+0x18) || read<BYTE>(entity+0x34)))return false;
    uintptr_t col=reinterpret_cast<GetCol>(0x535300)(entity);
    if(!col)return false;
    Shape s; s.lo=read<Vec>(col);s.hi=read<Vec>(col+12);
    uintptr_t m=read<uintptr_t>(entity+0x14);
    if(m) {s.x=read<Vec>(m);s.y=read<Vec>(m+0x10);s.z=read<Vec>(m+0x20);s.origin=read<Vec>(m+0x30);}
    else {
        float angle=read<float>(entity+0x10),c=std::cos(angle),a=std::sin(angle);
        s.x={c,a,0};s.y={-a,c,0};s.origin=read<Vec>(entity+4);
    }
    bool traffic=(read<unsigned char>(entity+0x36)&7)==2;
    Vec v=traffic?read<Vec>(entity+0x44)*50.f:Vec{};
    if(!bodyFrom(s,along,entity,traffic,v,b))return false;
    if(!traffic && !b.sign) {
        auto id=read<unsigned short>(entity+0x22);
        if(id<20000) {
            auto model=read<uintptr_t>(0xA9B0C8+4*id);
            unsigned special=model?(read<unsigned short>(model+0x12)>>11)&15:0;
            if(special==1 || special==2)b.family=Family::Tree;
        }
    }
    return true;
}
class StreamedWorld final:public World {
    uintptr_t car;
public:
    explicit StreamedWorld(uintptr_t playerCar):car(playerCar){}
    int nearby(Vec centre,float radius,Vec along,Body* out,int capacity) override {
        std::array<uintptr_t,512> found{};
        short n{}; reinterpret_cast<Nearby>(0x564A20)(centre,radius,true,&n,static_cast<short>(found.size()),
                                                     found.data(),true,true,false,true,true);
        int count=0;
        for(int i=0;i<n && i<int(found.size()) && count<capacity;i++) {
            auto entity=found[i];
            if(entity && entity!=car && describe(entity,along,out[count]))++count;
        }
        return count;
    }
    bool trace(Vec a,Vec b,Hit& h) override {
        ColPoint hit{};uintptr_t entity{};
        if(!reinterpret_cast<Trace>(0x56BA00)(a,b,hit,entity,true,false,false,true,true,false,false,false) || !entity)return false;
        Body dummy{}; bool compact=describe(entity,{1,0,0},dummy);
        h={hit.position,hit.normal,entity,compact,compact && dummy.sign};
        return true;
    }
};
float option(const wchar_t* section,const wchar_t* key,float fallback,float lo,float hi) {
    wchar_t buf[64];
    if(!GetPrivateProfileStringW(section,key,L"",buf,64,ini.c_str()) &&
       !GetPrivateProfileStringW(L"Whoosh",key,L"",buf,64,ini.c_str()))return fallback;
    wchar_t* tail=nullptr;float v=std::wcstof(buf,&tail);
    return tail!=buf && *tail==L'\0' && std::isfinite(v)?std::clamp(v,lo,hi):fallback;
}
void stopDriving();
void reload() {
    wchar_t key[32]{};
    if(!GetPrivateProfileStringW(L"General",L"ReloadKey",L"",key,32,ini.c_str()))
        GetPrivateProfileStringW(L"Whoosh",L"ReloadKey",L"120",key,32,ini.c_str());
    wchar_t* end{};
    bool legacy=key[0]==L'F' || key[0]==L'f';
    long code=std::wcstol(key+(legacy?1:0),&end,10);
    if(legacy)code=(code>=1 && code<=24)?VK_F1+code-1:0;
    reloadKey=!(*end) && code>=1 && code<=254 ? int(code) : VK_F9;
    enabled=option(L"General",L"Enabled",1,0,1)!=0;
    const int profile=int(option(L"Camera",L"CameraProfile",2,0,2));
    const bool hudShake=option(L"Camera",L"HUDShake",1,0,1)!=0;
    const bool windEnabled=option(L"WindAudio",L"WindEnabled",1,0,1)!=0;
    camera::select(enabled ? static_cast<camera::Profile>(profile) : camera::Profile::Native, option(L"Camera",L"CameraFOVFactor",1,.65f,1.5f),
        {option(L"Camera",L"CameraBoats",1,0,1)!=0, option(L"Camera",L"CameraMotorcycles",1,0,1)!=0,
        option(L"Camera",L"CameraPlanes",1,0,1)!=0, option(L"Camera",L"CameraHelicopters",1,0,1)!=0,
         option(L"Camera",L"CameraTrains",0,0,1)!=0}, option(L"Camera",L"CameraReturnTimeout",1,0,10),
        option(L"Camera",L"CameraReturnSpeed",2,.5f,5),hudShake);
    Settings cfg;
    cfg.minimumKmh=option(L"Whoosh",L"MinSpeedKmh",cfg.minimumKmh,5,300);
    cfg.fullKmh=std::max(cfg.minimumKmh+1,option(L"Whoosh",L"FullVolumeSpeedKmh",cfg.fullKmh,6,500));
    cfg.passMetres=option(L"Whoosh",L"PassDistance",cfg.passMetres,1.5f,15);
    cfg.roofMetres=option(L"Whoosh",L"TunnelRoofHeight",cfg.roofMetres,3,35);
    settingGain=option(L"Whoosh",L"VolumePercent",80,0,100)/100.f;
    detector.configure(cfg);
    visuals::Options fx;
    fx.blur=int(option(L"MotionBlur",L"MotionBlur",2,0,2));
    fx.flares=option(L"Trails",L"StreakFlares",1,0,1)!=0;
    fx.strength=option(L"MotionBlur",L"MotionBlurStrength",55,0,150)/100.f;
    fx.wind=option(L"Trails",L"WindTrails",1,0,1)!=0;
    fx.windStart=option(L"Trails",L"WindTrailSpeedKmh",120,18,450)/3.6f;
    fx.lights=option(L"Trails",L"NitroLightTrails",1,0,1)!=0;
    fx.windOpacity=option(L"Trails",L"WindTrailOpacityPercent",100,0,200)/100.f;
    visuals::configure(fx);
    float volume=option(L"WindAudio",L"WindVolumePercent",25,0,100)/100.f;
    audio::windVolume(windEnabled?volume:0);
    logging=option(L"General",L"Logging",0,0,1)!=0;
    if(logging) {
        static bool started=false;
        if(FILE* file=_wfopen((directory+L"\\Whoosh.SA.log").c_str(),started?L"ab":L"wb")) {
            started=true;
            std::fprintf(file,"Whoosh.SA 0.28 settings: Enabled=%d ReloadKey=%d CameraProfile=%d HUDShake=%d MotionBlur=%d Strength=%.0f WindTrails=%d Opacity=%.0f NitroLights=%d StreakFlares=%d WindAudio=%d Volume=%.0f PassDistance=%.1f RoofHeight=%.1f\n",
                enabled,reloadKey,profile,hudShake,fx.blur,fx.strength*100,fx.wind,fx.windOpacity*100,fx.lights,fx.flares,windEnabled,settingGain*100,cfg.passMetres,cfg.roofMetres);
            std::fclose(file);
        }
        lastReport=GetTickCount()-10000;
    }
    if(enabled)audio::start(directory);
    else {audio::level(0);stopDriving();}
}
void initialize() {
    initialized=true;
    wchar_t filename[32768];
    DWORD n=GetModuleFileNameW(module,filename,32768);
    if(!n || n>=32768)return;
    std::wstring full(filename,n);
    auto slash=full.find_last_of(L"\\/");
    if(slash==std::wstring::npos)return;
    directory=full.substr(0,slash);
    visuals::setTexturePath((directory+L"\\Whoosh.txd").c_str());
    ini=directory+L"\\Whoosh.SA.ini";
    reload();
}
void stopDriving() {
    visuals::driving(false);
    if(wasDriving) {audio::silence();detector.reset();wasDriving=false;lastVehicle=0;}
}
void service() {
    if(!initialized)initialize();
    if(ini.empty())return;
    DWORD pid{}; GetWindowThreadProcessId(GetForegroundWindow(),&pid);
    bool down=(GetAsyncKeyState(reloadKey)&0x8000)!=0;
    if(pid==GetCurrentProcessId() && down && !keyHeld) {
        reload();
        down=(GetAsyncKeyState(reloadKey)&0x8000)!=0;
    }
    keyHeld=down;
    reportStatus();
    if(!enabled)return;
    if(pid!=GetCurrentProcessId() || !gameplay()) {stopDriving();return;}
    auto car=reinterpret_cast<FindVehicle>(0x56E0D0)(-1,false);
    if(!car || read<uintptr_t>(car+0x460)!=reinterpret_cast<FindPed>(0x56E210)(-1)) {stopDriving();return;}
    unsigned type=read<unsigned>(car+0x594);
    if(type>2 && type!=9) {stopDriving();return;}
    DWORD t=read<DWORD>(0xB7CB84);
    if(wasDriving && (car!=lastVehicle || t-lastGameTime>300))stopDriving();
    constexpr uintptr_t hardware=0xB5F8B8;
    float v=read<float>(hardware+0x414)*read<float>(hardware+0x41c);
    if(!std::isfinite(v) || v<=0 || settingGain<=0) {
        audio::level(0);stopDriving();visuals::driving(true);return;
    }
    visuals::driving(true);
    wasDriving=true;lastVehicle=car;lastGameTime=t;
    audio::level(std::min(1.f,v)*settingGain);
    Vec vel=read<Vec>(car+0x44)*50.f;
    audio::speed(length(vel));
    StreamedWorld world(car);
    constexpr uintptr_t camera=0xB6F028;
    Vec right{vel.y,-vel.x,0};
    unsigned active=read<BYTE>(camera+0x59);
    if(active<3) {
        uintptr_t cam=camera+0x174+active*0x238;
        Vec forward=read<Vec>(cam+0x190),up=read<Vec>(cam+0x1b4);
        Vec axis{forward.y*up.z-forward.z*up.y,forward.z*up.x-forward.x*up.z,0};
        if(length(axis)>.05f)right=axis;
    }
    auto events=detector.update(world,{position(car),vel,right,t,car});
    passEvents+=events.count;
    for(int i=0;i<events.count;i++) {
        audio::play(events.list[i]);
    }
}
void __fastcall frame(void* engine,void*) {
    previousService(engine);
    try { service(); } catch(...) {enabled=false;camera::select(camera::Profile::Native,1.f);stopDriving();}
}
bool equals(uintptr_t p,const char* bytes,size_t n) {
    MEMORY_BASIC_INFORMATION info{};
    if(!VirtualQuery(reinterpret_cast<void*>(p),&info,sizeof(info)) || info.State!=MEM_COMMIT ||
       (info.Protect&(PAGE_GUARD|PAGE_NOACCESS)) || p+n>uintptr_t(info.BaseAddress)+info.RegionSize)
        return false;
    return std::memcmp(reinterpret_cast<const void*>(p),bytes,n)==0;
}
void install(HMODULE self) {
    if(reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr))!=0x400000)return;
    // Confirm the mapped executable layout before following any game pointers.
    auto* base=reinterpret_cast<const BYTE*>(0x400000);
    auto* dos=reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    if(dos->e_magic!=IMAGE_DOS_SIGNATURE)return;
    auto* nt=reinterpret_cast<const IMAGE_NT_HEADERS*>(base+dos->e_lfanew);
    if(nt->Signature!=IMAGE_NT_SIGNATURE || nt->OptionalHeader.SizeOfImage<=0x81c000)return;
    constexpr uintptr_t site=0x53e98b;
    if(!equals(site-5,"\xb9\x90\xbc\xb6\x00\xe8",6) ||
       !equals(site+5,"\x8b\x15\x38\x70\xc1\x00",6) ||
       !equals(0x56e0d0,"\x8b\x44\x24\x04\x85\xc0",6) ||
       !equals(0x56ba00,"\x83\xec\x60\x66\x81\x3d\x78\xcd\xb7\x00",10)) return;
    DWORD old;
    if(!VirtualProtect(reinterpret_cast<void*>(site+1),4,PAGE_EXECUTE_READWRITE,&old))return;
    HMODULE pinned{};
    if(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,
                          reinterpret_cast<LPCWSTR>(self),&pinned)) {
        module=self;
        visuals::install();
        camera::installGame();
        auto delta=read<int32_t>(site+1);
        previousService=reinterpret_cast<GameService>(site+5+delta);
        uint32_t displacement=reinterpret_cast<uintptr_t>(&frame)-(site+5);
        std::memcpy(reinterpret_cast<void*>(site+1),&displacement,4);
        FlushInstructionCache(GetCurrentProcess(),reinterpret_cast<void*>(site),5);
    }
    DWORD unused;VirtualProtect(reinterpret_cast<void*>(site+1),4,old,&unused);
}
}
BOOL WINAPI DllMain(HINSTANCE dll,DWORD reason,LPVOID) {
    if(reason==DLL_PROCESS_ATTACH)install(dll);
    return TRUE;
}
