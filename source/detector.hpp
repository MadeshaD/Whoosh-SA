#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace whoosh {
struct Vec { float x{}, y{}, z{}; };
inline Vec operator+(Vec a, Vec b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec operator-(Vec a, Vec b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec operator*(Vec a,float k) { return {a.x*k,a.y*k,a.z*k}; }
inline float dot(Vec a,Vec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline float length(Vec a) { return std::sqrt(dot(a,a)); }
inline float unitRange(float x) { return std::clamp(x,0.f,1.f); }
enum class Family { Post=1,Tree,Smack,Traffic,Bridge,Tunnel,Exit };
enum class Kind { Small, Large, Tunnel };
struct Settings {
    float minimumKmh=64.8f, fullKmh=162.f, passMetres=10, roofMetres=22;
};
struct Body {
    uintptr_t id{};
    Vec centre{}, velocity{};
    float halfAcross{}, halfAlong{}, lowZ{}, highZ{};
    bool large{}; Family family=Family::Post; bool sign{};
};
struct Hit { Vec point{}, normal{}; uintptr_t id{}; bool compact{},ignoreRoof{}; };
class World {
public:
    virtual int nearby(Vec centre,float radius,Vec forward,Body* out,int capacity)=0;
    virtual bool trace(Vec from,Vec to,Hit& hit)=0;
    virtual ~World()=default;
};
struct PassMotion {
    float distance{},relativeSpeed{},secondsToPass{},headingPan{};
};
struct Event { Kind kind{}; float gain{},pan{}; uintptr_t id{}; PassMotion motion{}; Family family=Family::Post; float intensity{}; bool panel{}; };
struct Events { std::array<Event,10> list{}; int count{}; };
struct Frame { Vec position{},velocity{},cameraRight{}; uint32_t timeMs{}; uintptr_t vehicle{}; };
class Detector {
public:
    explicit Detector(Settings settings={}) : cfg(settings) {}
    Events update(World& world,const Frame& frame);
    void configure(Settings settings) { cfg=settings; }
    void reset();
private:
    struct Side { bool wall{}; float gap{}; float distance{}; Vec normal{}; };
    struct Recent { uintptr_t id{}; Vec at{}; uint32_t when{}; };
    Settings cfg;
    std::array<Side,2> sides{};
    std::array<Recent,48> recent{};
    size_t next{};
    bool running{},covered{}; Family portal=Family::Tunnel;
    float openGap{};
    uint32_t lastTime{};
    uintptr_t vehicle{};
    Vec lastPosition{},lastDirection{};
};
}
