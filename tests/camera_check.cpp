#include "../source/chase_collision.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
using namespace whoosh;
using namespace whoosh::camera;

namespace {
bool near(float a,float b,float tolerance=.0001f){return std::abs(a-b)<tolerance;}
Input drive(Profile p,unsigned zoom,int fps,float speed) {
    Input i;
    i.returnTimeout=.35f; i.returnSpeed=5.f; i.profile=p; i.zoom=zoom; i.velocity={0,speed,0}; i.topSpeed=60;
    i.grounded=true; i.dt=1.f/fps; return i;
}
void checkModes() {
    for(auto p:{Profile::MostWanted,Profile::Carbon}) {
        for(unsigned z=0;z<=3;++z) {
            unsigned m=z ? 18 : 16;
            assert(supported(m,z));
            assert(!supported(z ? 16 : 18,z));
            assert(drawFov(Profile::Native,m,z,64)==64);
            assert(drawFov(p,4,z,64)==64);
            assert(drawFov(p,m,z,64)==(!z ? 83 : p==Profile::Carbon ? 71 : z==3 ? 90 : 78));
        }
    }
}
void regulation(int fps) {
    for(auto p:{Profile::MostWanted,Profile::Carbon}) {
        for(unsigned zoom=0;zoom<=3;++zoom) {
            Controller c;
            auto i=drive(p,zoom,fps,0);
            Response r=c.step(i);
            if(zoom) {
                const float far=p==Profile::Carbon ? 5.9f : 5.436f;
                assert(near(-r.lag,4.7f+(far-4.7f)*(zoom-1)*.5f));
                assert(near(r.eye,0) && near(r.target,0) && near(r.targetHeight,0));
            } else assert(near(r.eye,0) && near(r.targetHeight,-.0261859f));
            for(int f=0;f<2*fps;++f) {
                i.velocity.y+=10*i.dt;
                r=c.step(i);
                assert(near(length(r.shake),0));
            }
            if(zoom) assert(r.eye<-.75f && r.target<-.06f);
            // Start from steady-speed cruise so the brake response is not
            // confused with unwinding the earlier acceleration segment.
            Controller braking;
            i.velocity.y=38; braking.step(i);
            i.braking=true;
            for(int f=0;f<fps;++f) {
                i.velocity.y-=18*i.dt;
                r=braking.step(i);
            }
            if(zoom) {
                if(p==Profile::Carbon) assert(r.targetHeight<-.25f && r.eye<=.01f);
                else assert(near(r.targetHeight,0) && r.eye>0);
            }
            i=drive(p,zoom,fps,60);
            Controller top;
            r=top.step(i);
            if(zoom) assert(near(r.eye,r.lag*.2f));
            else assert(near(r.eye,0));
            i.dt=.5f;
            assert(near(top.step(i).eye,0));
        }
    }
}
void autoFollowAndMouse(int fps) {
    for(auto p:{Profile::MostWanted,Profile::Carbon}) {
        auto i=drive(p,2,fps,0);
        i.velocity={20,20,0}; i.viewFront=i.forward;
        Controller c;
        auto corner=c.step(i);
        assert(near(corner.viewFront.x,.7071068f));
        i.manualLook=true;
        const float mouseYaw=.12f;
        i.viewFront={std::sin(mouseYaw),std::cos(mouseYaw),0};
        auto manual=c.step(i);
        assert(near(std::atan2(manual.viewFront.x,manual.viewFront.y),.78539816f+mouseYaw));
        i.viewFront={-1,0,0};
        for(int f=0;f<4*fps;++f) manual=c.step(i);
        assert(near(std::atan2(manual.viewFront.x,manual.viewFront.y),-.78539816f));
        i.manualLook=false;
        auto released=c.step(i);
        assert(dot(manual.viewFront,released.viewFront)>.9999f);
        for(int f=0;f<3*fps;++f) released=c.step(i);
        assert(dot(corner.viewFront,released.viewFront)>.999f);
        i=drive(p,3,fps,-25);
        assert(c.step(i).viewFront.y>.99f); // reversing keeps the nose ahead
    }
}
void returnSettings(int fps) {
    auto i=drive(Profile::MostWanted,2,fps,20);
    Controller c;
    c.step(i);
    i.returnTimeout=1; i.returnSpeed=2;
    i.manualLook=true; i.viewFront={0,.01f,.99995f};
    const auto limited=c.step(i);
    assert(near(std::asin(limited.viewFront.z),.349066f));
    i.viewFront={1,0,0};
    const auto held=c.step(i);
    i.manualLook=false; i.viewFront={0,1,0};
    for(int frame=0;frame<fps-1;++frame)
        assert(dot(c.step(i).viewFront,held.viewFront)>.99999f);
    for(int frame=0;frame<fps*4;++frame)c.step(i);
    assert(c.step(i).viewFront.y>.999f);
}
void returnDuringTurn(int fps) {
    for(auto profile:{Profile::MostWanted,Profile::Carbon}) {
        Controller orbit,baseline;
        auto i=drive(profile,2,fps,20);
        i.returnTimeout=0; i.returnSpeed=2;
        for(int frame=0;frame<fps*6;++frame) {
            const float angle=frame*i.dt*1.4f;
            i.forward={std::sin(angle),std::cos(angle),0};
            i.velocity=i.forward*20.f;
            i.viewFront=i.forward;
            i.manualLook=false;
            auto reference=baseline.step(i);
            if(frame==1) {i.manualLook=true; i.viewFront={1,0,0};}
            auto actual=orbit.step(i);
            if(frame>fps*4) assert(length(actual.viewFront-reference.viewFront)<1.e-6f);
        }
    }
}
void chaseCollision() {
    const ChaseLens lens{.7f,1.12f,.63f};
    const ChaseBody body{{0,0,.62f},{1,0,0},{0,1,0},{0,0,1},
                         {-1.12f,-2.35f,-.55f},{1.14f,2.48f,.62f},true};
    const Vec pivot{0,0,1.16f};
    Vehicle vehicle; vehicle.centre=body.centre;
    Response motion; motion.lag=-5.4f;
    // The game has shortened its own camera while an unrelated world-space
    // mouse orbit still has a clear view. Use the pre-collision stock zoom.
    Pose stock{pivot+Vec{0,-.18f,0},{0,1,0},{0,0,1}};
    motion.viewFront={1,0,0};
    const auto clear=place(stock,vehicle,motion,false,{}, {pivot,8.f});
    assert(near(length(clear.eye-pivot),8.f));
    assert(near(clear.eye.x,-8.f));
    for(float winding:{-1.f,1.f}) for(float slope:{-.12f,0.f,.12f}) {
        auto floor=[&](Vec a,Vec b,ChaseHit& hit) {
            const float ad=a.z-slope*a.y, bd=b.z-slope*b.y;
            if(ad<0 || bd>0 || ad==bd) return false;
            hit={a+(b-a)*(ad/(ad-bd)),Vec{0,-slope,1}*(winding/std::hypot(slope,1.f))};
            return true;
        };
        ChaseTravel travel;
        for(int frame=0;frame<480;++frame) {
            const float yaw=frame*.032f,pitch=.43f*std::sin(frame*.024f)-.11f;
            motion.viewFront={std::sin(yaw)*std::cos(pitch),std::cos(yaw)*std::cos(pitch),std::sin(pitch)};
            auto p=place(stock,vehicle,motion,false,{}, {pivot,8.f});
            const auto actual=resolveChase(p,body,lens,travel,1.f/60,floor);
            assert(length(actual.position-pivot)>5.f);
            assert(actual.position.z>=slope*actual.position.y);
            for(Vec corner:nearCorners(actual,lens)) {
                const Vec q=actual.position+corner;
                assert(q.z>=slope*q.y-.001f);
            }
        }
    }
    // Check the frustum against an off-axis pillar, then remove it. Recovery
    // must extend the arm progressively without changing the user's bearing.
    bool obstacle=true;
    auto pillar=[&](Vec a,Vec b,ChaseHit& contact) {
        if(!obstacle || a.y< -3 || b.y>= -3 || a.y==b.y) return false;
        const Vec q=a+(b-a)*((-3.f-a.y)/(b.y-a.y));
        if(q.x<.19f || q.x>.9f || q.z<.1f || q.z>3.f) return false;
        contact={q,{0,1,0}}; return true;
    };
    ChaseBody none{};
    Placement p{{0,0,1},{0,-6,1},{0,0,1},{0,0,1}};
    ChaseTravel arm;
    float radius=length(resolveChase(p,none,lens,arm,.02f,pillar).position-p.anchor);
    assert(radius>3.f && radius<4.f);
    obstacle=false;
    for(int i=0;i<60;i++) {
        float next=length(resolveChase(p,none,lens,arm,.02f,pillar).position-p.anchor);
        assert(next>=radius-.002f && next-radius<.6f);
        radius=next;
    }
    assert(near(radius,6.f,.012f));
}
void impulse(int fps) {
    auto i=drive(Profile::Carbon,2,fps,8);
    Controller c; c.step(i);
    i.contact=true;
    i.velocity.y-=600/.7f*i.dt;
    Response r=c.step(i);
    assert(near(length(r.shake),.4f,.0005f) && near(r.hudScale,1.08f,.0001f));
    bool gap=false, secondPulse=false;
    for(int frame=1;frame<=fps;++frame) {
        r=c.step(i);
        if(length(r.shake)<1e-6f) gap=true;
        if(gap && length(r.shake)>.01f) secondPulse=true;
    }
    assert(gap && secondPulse && near(length(r.shake),0) && near(r.hudScale,1));
    // View changes, the first driving frame, or switching to native must
    // never inherit an impulse from the previous vehicle/view.
    i.velocity.y+=100;
    r=c.step(i);assert(length(r.shake)>0);
    i.zoom=0; r=c.step(i);assert(near(length(r.shake),0));
    i.profile=Profile::Native; r=c.step(i);assert(near(r.hudScale,1));
}
void parkingAndLightContact(int fps) {
    Controller c;
    auto i=drive(Profile::Carbon,1,fps,0);
    c.step(i);
    i.manualLook=true; i.viewFront={1,0,0};
    c.step(i);
    i.manualLook=false; i.viewFront={0,1,0};
    for(int f=0;f<fps*4;++f) assert(c.step(i).viewFront.x>.999f);
    i.contact=true; i.velocity.y=1;
    for(int f=0;f<fps*3;++f) {
        auto r=c.step(i);
        assert(length(r.shake)==0);
        if(f==fps*3-1) assert(r.viewFront.y>.999f);
    }
}
void levelOrbit() {
    Vec lo{-1,-2,-.6f},hi{1,2,1};
    assert(near(bodyClearance({0,-1,0},{0,-1,0},lo,hi,.3f),1.32f));
    assert(near(bodyClearance({0,-3,0},{0,-1,0},lo,hi,.3f),0));
    Pose stock{{0,-5,1},{0,1,0},{.6f,0,.8f}};
    Vehicle car;
    Response r;
    for(float angle:{-2.f,-1.f,0.f,1.f,2.f}) {
        r.viewFront={std::sin(angle),std::cos(angle),0};
        const auto p=place(stock,car,r,false);
        const auto pose=orient(p,p.eye);
        assert(pose.up.z>.999f);
    }
    const auto front=blendBearing({0,1,0},{0,-1,0},.5f);
    assert(near(length(front),1) && std::abs(front.x)>.999f);
    assert(supported(22,2));
}
void roadNoise() {
    for(float speed:{44.9f,45.f,60.f}) {
        for(auto p:{Profile::MostWanted,Profile::Carbon}) {
            auto i=drive(p,3,60,speed);
            i.road=wheelSurface(6,2); // gravel
            Controller c;
            float maxHud=1, motion=0; bool varies=false;
            float previous=0;
            for(int f=0;f<120;++f) {
                auto r=c.step(i);
                motion=std::max(motion,std::abs(r.vibration[2]));
                maxHud=std::max(maxHud,r.hudScale);
                varies|=(f>0 && std::abs(previous-r.vibration[1])>1.e-6f);
                previous=r.vibration[1];
            }
            assert(varies && motion>.0001f);
            assert((maxHud>1.0001f)==(p==Profile::Carbon && speed>=45));
        }
    }
    auto i=drive(Profile::Carbon,1,60,60);
    Controller a,b;
    auto smooth=a.step(i);
    i.road=wheelSurface(5,1); auto cobble=b.step(i);
    assert(!near(smooth.vibration[2],cobble.vibration[2],1.e-6f));
    i.velocity={}; Controller rest;
    assert((rest.step(i).vibration==std::array<float,4>{}));
}
void orbitAndHud() {
    Vehicle v; v.centre={20,30,5}; v.roof=1; v.rearExtent=2;
    Response r; r.height=1.62f; r.lag=-4.7f;
    for(Vec dir:{Vec{0,1,0},Vec{1,0,0},Vec{0,-1,0},Vec{0,.8f,-.6f}}) {
        Pose native;native.front=dir; native.position=v.centre-dir*4.7f;
        auto p=place(native,v,r,false); auto view=orient(p,p.eye);
        assert(near(dot(view.front,dir),1));
        assert(near(dot(view.front,view.up),0));
        assert(near(length(view.position-p.anchor),4.7f));
        auto hit=[&](Vec from,Vec,Vec& result){result=from-dir*2.f;return true;};
        const Vec clip=limitTravel(p.anchor,p.eye,.24f,hit);
        assert(near(length(clip-p.anchor),1.76f));
        assert(near(dot(orient(p,clip).front,dir),1));
    }
    // Rotating an off-axis far view must preserve its orbit radius at steep pitches.
    Pose far{{20,22,9},{0,1,0},{0,0,1}};
    for(float pitch=-1.35f;pitch<1.36f;pitch+=.05f) {
        r.viewFront={0,std::cos(pitch),std::sin(pitch)};
        auto p=place(far,v,r,false);
        assert(near(length(p.eye-v.centre),length(far.position-v.centre)));
        assert(length(p.anchor-v.centre)<1e-5f);
    }
    const HudVertex src[2]={{0,0,.2f,.5f,0xff112233,.4f,.7f},{960,540,.9f,1,0xff001122,0,1}};
    HudVertex copy[2]{};
    scaleHudVertices(src,copy,2,960,540,1.05f);
    assert(near(copy[0].x,-48) && near(copy[0].y,-27));
    assert(!std::memcmp(src+1,copy+1,sizeof(HudVertex)));
    assert(!std::memcmp(&src[0].z,&copy[0].z,sizeof(HudVertex)-8));
    assert(src[0].x==0 && src[0].y==0);
}
}
int main() {
    using T=VehicleType;
    for(unsigned mask=0;mask<32;++mask) {
        VehicleOptions o{bool(mask&1),bool(mask&2),bool(mask&4),bool(mask&8),bool(mask&16)};
        assert(o.allows(T::Motorcycle)==o.motorcycles && o.allows(T::Bicycle)==o.motorcycles);
        assert(o.allows(T::Helicopter)==o.helicopters && o.allows(T::FakeHelicopter)==o.helicopters);
        assert(o.allows(T::Plane)==o.planes && o.allows(T::FakePlane)==o.planes);
        assert(o.allows(T::Train)==o.trains && o.allows(T::Boat)==o.boats);
        assert(o.allows(T::MonsterTruck) && o.allows(T::Quad) && o.allows(T::Automobile));
        assert(!o.allows(T::Trailer) && !o.allows(static_cast<T>(255)));
    }
    assert(!cameraHandbrake(T::MonsterTruck,true) && usesRoadBrakes(T::MonsterTruck));
    assert(cameraHandbrake(T::Automobile,true) && !cameraHandbrake(T::Automobile,false));
    assert(!cameraHandbrake(T::Helicopter,true) && !hasCarWheels(T::Bicycle));

    chaseCollision(); checkModes(); levelOrbit(); roadNoise(); orbitAndHud();
    for(int fps:{25,30,60,144}) {returnDuringTurn(fps); returnSettings(fps); parkingAndLightContact(fps); regulation(fps); impulse(fps); autoFollowAndMouse(fps);}
    std::puts("Four views, two driving profiles, stock bypass, braking, impact envelope, road noise, orbit, clipping and HUD verified.");
}
