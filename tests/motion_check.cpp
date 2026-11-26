#include "../source/motion.hpp"
#include <cassert>
using namespace whoosh;
using namespace whoosh::visuals;
int main(){
    for(unsigned i=0;i<8;++i) {
        auto center=nitrousRadialTap({.5f,.5f},.18f,i);
        assert(center.x==.5f && center.y==.5f);
        auto off=nitrousRadialTap({0,1},0,i);
        assert(off.x==0 && off.y==1);
    }
    auto corner=nitrousRadialTap({0,1},.18f,7);
    assert(corner.x>.08f && corner.y<.92f);
    float sum=0;
    for(float w:carbonWeights) sum+=w;
    assert(std::abs(sum-1)<1e-6f);
    // Trace the actual 8-bit blend coefficients with one nonzero input tap.
    for(bool carbon:{false,true}) {
        assert(spatialAlpha(carbon,0)==255);
        for(unsigned impulse=0;impulse<8;++impulse) {
            long pixel=217; // arbitrary previous target
            for(unsigned tap=0;tap<8;++tap) {
                auto a=spatialAlpha(carbon,tap);
                pixel=std::lround((a*(tap==impulse ? 255.f : 0.f)+(255-a)*pixel)/255.f);
            }
            constexpr float mw[]{1,2,2,3,3,2,2,1};
            float weight=carbon ? carbonWeights[impulse] : mw[impulse]/16;
            assert(std::abs(pixel-255*weight)<=2);
        }
    }
    // Filter a one-pixel stripe at high speed. Ignore sub-visible tails.
    for(bool carbon:{false,true}) {
        std::array<float,512> input{},output{};
        input[256]=1;
        for(unsigned pass=1;pass<=(carbon?3u:2u);++pass) {
            for(int x=0;x<512;++x) {
                output[x]=0;
                for(unsigned tap=0;tap<8;++tap) {
                    float u=spatialTap({x/512.f,.5f},{1,0,0},.04f,carbon,pass,tap,512,256).x*512;
                    int a=std::clamp(int(std::floor(u)),0,510);
                    float f=unitRange(u-a);
                    constexpr float mw[]{1,2,2,3,3,2,2,1};
                    output[x]+=(input[a]*(1-f)+input[a+1]*f)*(carbon?carbonWeights[tap]:mw[tap]/16);
                }
            }
            input=output;
        }
        float energy=0;int runs=0,count=0;bool inside=false;
        for(float v:input) {energy+=v;if(v>.001f){runs+=!inside;++count;}inside=v>.001f;}
        assert(std::abs(energy-1)<1e-4f && runs==1 && count>8);
    }
    assert(blurDrive(true,0,1)==0 && blurDrive(false,0,1)==0);
    // Perspective view in normalized raster coordinates, looking down world Y.
    Projection view{{1,0,0},{.5f,.5f,1},{0,-1,0},{}};
    UV c{.5f,.5f}, edge{.9f,.5f};
    auto forward=view.vector({0,1,0});
    auto zoom=sampleMotion(edge,forward,.1f);
    assert(zoom.x<edge.x && zoom.y==edge.y);
    assert(sampleMotion(c,forward,.1f).x==c.x);
    assert(sampleMotion(edge,view.vector({0,-1,0}),.1f).x>edge.x);
    for(float dir:{-1.f,1.f}){
        auto side=view.vector({dir,0,0});
        auto a=sampleMotion(c,side,.1f),b=sampleMotion(edge,side,.1f);
        assert((a.x-c.x)*dir>0 && a.y==c.y && b.y==edge.y);
        assert(std::abs((a.x-c.x)-(b.x-edge.x))<1e-6f);
        assert(std::abs(sampleMotion(c,side,.2f).x-c.x)>std::abs(a.x-c.x));
    }
}
