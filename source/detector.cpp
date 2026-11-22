#include "detector.hpp"

namespace whoosh {
void Detector::reset() {
    if(running) *this=Detector(cfg);
}

Events Detector::update(World& world,const Frame& frame) {
    Events result{};
    Vec horizontal{frame.velocity.x,frame.velocity.y,0};
    float speed=length(horizontal),kmh=length(frame.velocity)*3.6f;
    if(!frame.vehicle || !std::isfinite(kmh) || speed<3.f || kmh<=cfg.minimumKmh) {
        reset(); return result;
    }
    Vec along=horizontal*(1.f/speed),right{along.y,-along.x,0};
    Vec ear{frame.cameraRight.x,frame.cameraRight.y,0};
    float earLength=length(ear);
    ear=earLength>.1f?ear*(1.f/earLength):right;
    const uint32_t elapsed=frame.timeMs-lastTime;
    float moved=length(frame.position-lastPosition);
    if(running && (vehicle!=frame.vehicle || elapsed>300 || moved>speed*elapsed*.001f+7.f ||
                   dot(along,lastDirection)<.25f)) reset();
    if(running && (elapsed<14 || (elapsed<40 && moved<.65f))) return result;
    const bool prime=!running;
    if(prime) moved=0;
    running=true; lastTime=frame.timeMs; lastPosition=frame.position;
    vehicle=frame.vehicle; lastDirection=along;
    // Anticipate the attack of the one-shot and the short device buffer.
    const float lead=std::clamp(speed*.30f,3.f,18.f);
    const Vec grade=along+Vec{0,0,std::clamp(frame.velocity.z/speed,-.7f,.7f)};
    const Vec probe=frame.position+grade*lead+Vec{0,0,.4f};
    // Most Wanted maps 18..45 m/s linearly to the normalized drive-by level.
    const float speedGain=unitRange((kmh-cfg.minimumKmh)/std::max(1.f,cfg.fullKmh-cfg.minimumKmh));
    auto roof=[&](Vec p) {
        Hit h;
        const Vec top=p+Vec{0,0,cfg.roofMetres};
        auto valid=[&]() {return !h.ignoreRoof && std::abs(h.normal.z)>.45f && h.point.z>p.z+2.0f;};
        bool bottom=world.trace(p,top,h) && valid();
        if(bottom && h.normal.z<0)return true;
        Hit lower=h;
        // Some bridge decks expose only their top face to collision queries.
        if(!world.trace(top,p+Vec{0,0,1.1f},h) || h.ignoreRoof)return false;
        // A top face alone may be the road ahead on an incline. Only accept
        // it when a second surface supports the lane beneath the probe.
        if(bottom && h.id==lower.id && h.point.z-lower.point.z>.45f)return true;
        if(!valid())return false;
        Hit floor;
        return world.trace(p+Vec{0,0,.4f},p-Vec{0,0,9.f},floor) && std::abs(floor.normal.z)>.45f;
    };
    // Sweep the lane over the distance since the last check. Short arches can
    // lie entirely between two frame positions on a fast approach.
    Vec first=probe-grade*std::min(moved,7.f);
    int steps=std::max(1,int(std::ceil(std::min(moved,7.f)/1.35f)));
    auto emitPortal=[&](Family family,float seconds) {
        Event e{Kind::Tunnel,speedGain*(family==Family::Exit?1.05f:1.65f),0,0,{0,0,seconds,0}};
        e.family=family;e.intensity=speedGain;
        result.list[result.count++]=e;
    };
    for(int i=1;i<=steps;i++) {
        Vec pos=first+(probe-first)*(float(i)/steps);
        bool centre=roof(pos);
        bool overhead=centre || (roof(pos+right*.95f) && roof(pos-right*.95f));
        if(overhead) {
            openGap=0;
            if(!covered) {
                // Check width and at least a short stretch of overhead cover.
                bool width=roof(pos+right*1.4f) && roof(pos-right*1.4f);
                auto spansDeck=[&](Vec step) {
                    return roof(pos+step*2.f) && roof(pos+step*4.f) && roof(pos+step*6.f);
                };
                bool depth=width && (spansDeck(grade) || spansDeck(grade*(-1.f)) ||
                    (roof(pos-grade*3.f) && roof(pos+grade*3.f) &&
                     roof(pos-grade*1.5f) && roof(pos+grade*1.5f)));
                if(width && depth) {
                    covered=true;
                    portal=Family::Bridge;
                    Vec inside=pos+along*8.f;
                    if(roof(inside)) {
                        Hit side;
                        auto hasWall=[&](float metres) {
                            return world.trace(inside,inside+right*metres,side) && std::abs(side.normal.z)<.7f;
                        };
                        if(hasWall(22.f) || hasWall(-22.f))portal=Family::Tunnel;
                    }
                    if(!prime)emitPortal(portal,std::max(0.f,dot(pos-frame.position,along)/speed));
                }
            }
        } else if(covered && (openGap+=moved/steps)>=4.f) {
            covered=false;openGap=0;if(!prime)emitPortal(Family::Exit,std::max(0.f,(lead-4.f)/speed));
        }
    }
    struct Choice { Event e{}; Vec from{},target{},centre{}; bool compact{}; };
    std::array<Choice,258> candidates{};
    int count=0;
    auto propose=[&](Kind kind,float gap,Vec offset,uintptr_t id,float scale) -> Choice* {
        float near=unitRange(1.f-gap/cfg.passMetres);
        float gain=std::sqrt(speedGain)*near*(.35f+.65f*near)*scale;
        if(gain<.006f || count==int(candidates.size()))return nullptr;
        float pan=std::clamp(dot(ear,offset)/std::max(.05f,length(offset)),-.95f,.95f);
        Choice& c=candidates[count++];c.e={kind,gain,pan,id};c.e.intensity=speedGain;return &c;
    };
    for(int s=0;s<2;s++) {
        Vec edge=right*(s?1.f:-1.f); Hit h; Side& state=sides[s];
        auto sideHit=[&](Vec start) {
            return world.trace(start,start+edge*cfg.passMetres,h) &&
                   !h.compact && std::abs(h.normal.z)<.68f;
        };
        bool wall=sideHit(probe) || sideHit(probe-Vec{0,0,.72f});
        if(wall) {
            float gap=std::abs(dot(h.point-probe,edge));
            bool newFace=!state.wall || gap+1.5f<state.distance || dot(h.normal,state.normal)<.62f;
            if(!prime && newFace && !covered) {
                if(auto* c=propose(Kind::Large,gap,edge,h.id,1.f)) {
                    c->e.motion={std::max(.5f,gap),speed,lead/speed,dot(ear,along)};
                    c->e.family=Family::Smack;
                }
            }
            state.wall=true;state.distance=gap;state.normal=h.normal;state.gap=0;
        } else if(state.wall && (state.gap+=moved)>=2.5f)state={};
    }
    // Candidate centres cover the swept interval even when a thin post falls
    // between frames. No game pointers survive as dereferenceable references.
    constexpr int capacity=256;
    std::array<Body,capacity> bodies;
    Vec centre=frame.position+along*(lead*.5f);
    int n=world.nearby(centre,lead+cfg.passMetres+7.f,along,bodies.data(),capacity);
    for(int i=0;!prime && i<std::min(n,capacity);++i) {
        const auto& b=bodies[i]; Vec delta=b.centre-frame.position;
        float verticalReach=b.sign?cfg.roofMetres:(b.family==Family::Post?2.2f:.65f);
        if(frame.position.z+verticalReach<b.lowZ || frame.position.z-.6f>b.highZ)continue;
        Vec relative=horizontal-Vec{b.velocity.x,b.velocity.y,0};
        float rate=length(relative);
        if(rate<6.f)continue;
        Vec path=relative*(1.f/rate),perp{path.y,-path.x,0};
        float ahead=dot(delta,path),lateral=dot(delta,perp);
        // The larger footprint of traffic and the player's body must not
        // turn a head-on collision or a lead car into a roadside fly-by.
        if(!b.sign && std::abs(lateral)<.55f+.50f*std::min(2.0f,b.halfAcross))continue;
        float gap=std::max(0.f,std::abs(lateral)-(b.sign?b.halfAcross:std::min(b.halfAcross,1.25f)));
        float reach=std::min(22.f,rate*.30f)+std::min(2.5f,b.halfAlong);
        if(gap>=cfg.passMetres || ahead>reach || ahead< -std::max(1.f,rate*elapsed*.001f))continue;
        bool repeated=false;
        for(const auto& seen:recent) {
            if(seen.id==b.id && frame.timeMs-seen.when<1600 && length(b.centre-seen.at)<32.f) {
                repeated=true; break;
            }
        }
        if(repeated)continue;
        Vec offset=perp*lateral;
        if(auto* c=propose(b.large?Kind::Large:Kind::Small,gap,offset,b.id,std::min(1.f,rate/16.f))) {
            float timeToPass=std::clamp(ahead/rate,0.f,.5f);
            c->e.motion={std::abs(lateral),rate,timeToPass,dot(ear,path)};
            c->e.panel=b.sign;c->e.family=b.family;c->e.intensity=unitRange((rate*3.6f-cfg.minimumKmh)/(cfg.fullKmh-cfg.minimumKmh));
            c->from=frame.position+horizontal*timeToPass+Vec{0,0,.25f};
            c->target=b.centre+b.velocity*timeToPass;c->target.z=c->from.z;
            // A median may hide the base while most of the post remains exposed.
            if(b.family==Family::Post || b.family==Family::Tree)
                c->target.z=std::clamp(c->from.z+2.f,b.lowZ,b.highZ);
            c->centre=b.centre;c->compact=true;
        }
    }
    std::sort(candidates.begin(),candidates.begin()+count,[](const Choice& a,const Choice& b) {
        return a.e.motion.secondsToPass<b.e.motion.secondsToPass;
    });
    for(int i=0;i<count && result.count<int(result.list.size());i++) {
        const auto& c=candidates[i];
        bool duplicate=false;
        for(int j=0;j<result.count;j++)if(c.e.id==result.list[j].id)duplicate=true;
        if(duplicate)continue;
        if(c.compact) {
            Hit block;
            if(world.trace(c.from,c.target,block) && block.id!=c.e.id &&
               !block.compact && length(block.point-c.target)>.4f) {
                Vec upper=c.from+Vec{0,0,1.f};
                if(c.e.family!=Family::Post || (world.trace(upper,c.target,block) && block.id!=c.e.id && !block.compact))continue;
            }
            recent[next]={c.e.id,c.centre,frame.timeMs};next=(next+1)%recent.size();
        }
        result.list[result.count++]=c.e;
    }
    return result;
}
}
