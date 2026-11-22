#include "../source/detector.hpp"
#include <cassert>
#include <cstdio>
using namespace whoosh;
class Street:public World {
public:
    Vec start{25,3,1},velocity{};
    float t{},roofEnd{},wallY{},ceilingZ=6;bool topOnly=false;
    bool hasCar=true, pair=false;
    int nearby(Vec,float,Vec,Body* out,int) override {
        if(!hasCar)return 0;
        out[0]={9,start+velocity*t,velocity,1,2,0,2,true};if(pair){out[1]=out[0];out[1].id=13;out[1].centre.x+=2;return 2;}
        return 1;
    }
    bool trace(Vec a,Vec b,Hit& h) override {
        if((topOnly?(a.z>ceilingZ && b.z<ceilingZ):(b.z>ceilingZ && a.z<ceilingZ)) && a.x>=20 && a.x<=roofEnd) {
            h={{a.x,a.y,ceilingZ},{0,0,-1},4,false};return true;
        }
        if(a.z>0 && b.z<0){h={{a.x,a.y,0},{0,0,1},7,false};return true;}
        if(wallY && (wallY-a.y)*(wallY-b.y)<0) {
            h={a+(b-a)*((wallY-a.y)/(b.y-a.y)),{0,-1,0},5,false};return true;
        }
        return false;
    }
};
struct Counts {int bridges{},exits{},cars{},roofs{},second{};float level{},pan{};};
Counts drive(Street& world) {
    Detector detector;Counts c;
    for(int i=0;i<240;i++) {
        world.t=i/60.f;
        auto events=detector.update(world,{{30*world.t,0,1},{30,0,0},{0,-1,0},uint32_t(5000+i*1000/60),1});
        for(int j=0;j<events.count;j++) {
            auto e=events.list[j];
            if(e.id==13)++c.second;
            if(e.id==9){++c.cars;c.pan=e.pan;c.level=e.gain;}
            if(e.family==Family::Tunnel)++c.roofs;
            if(e.family==Family::Exit)++c.exits;
            if(e.family==Family::Bridge)++c.bridges;
        }
    }
    return c;
}
int main() {
    Street scene;
    auto stopped=drive(scene);assert(stopped.cars==1 && stopped.pan<0);
    scene.start.y=5;auto distant=drive(scene);assert(distant.cars==1 && distant.level<stopped.level*.65f);scene.start.y=3;
    scene.pair=true;auto closePair=drive(scene);assert(closePair.cars==1 && closePair.second==1);scene.pair=false;
    scene.velocity={-20,0,0};assert(drive(scene).cars==1);
    scene.velocity={30,0,0};assert(drive(scene).cars==0);
    scene.velocity={0,0,0};scene.wallY=1.2f;assert(drive(scene).cars==0);
    scene.wallY=0;scene.start.y=0;assert(drive(scene).cars==0);
    scene.hasCar=false;scene.roofEnd=21;auto beam=drive(scene);assert(beam.roofs==0 && beam.bridges==0);
    scene.roofEnd=24;auto arch=drive(scene);assert(!arch.bridges && !arch.exits);
    scene.roofEnd=95;scene.wallY=-12;assert(drive(scene).roofs==1);
    scene.wallY=0;scene.ceilingZ=19;scene.topOnly=true;assert(drive(scene).bridges==1);
    std::puts("Parked, oncoming, parallel, head-on, hidden, beam and tunnel checks passed.");
}
