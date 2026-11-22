// Offline pass-by checks on a GTA Z-up OBJ collision scene. Build together
// with source/detector.cpp. No game or audio device is needed.
#include "../source/geometry.hpp"
#include "../source/camera.hpp"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
using namespace whoosh;
struct Triangle { int a,b,c; };
struct Mesh {
    std::string name;
    Vec lo{1e9,1e9,1e9},hi{-1e9,-1e9,-1e9};
    std::vector<Triangle> tris;
    std::vector<int> vertices;
    Shape shape;
};
Vec cross(Vec a,Vec b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
Vec low(Vec a,Vec b) {return {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)};}
Vec high(Vec a,Vec b) {return {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};}
bool overlap(Vec l,Vec h,Vec l2,Vec h2) {
    return l.x<=h2.x && h.x>=l2.x && l.y<=h2.y && h.y>=l2.y && l.z<=h2.z && h.z>=l2.z;
}
class Scene:public World {
public:
    std::vector<Mesh> meshes;
    std::vector<Vec> verts;
    unsigned rays{},queries{};
    explicit Scene(const char* path) {
        std::ifstream f(path); std::string line;
        if(!f)throw std::runtime_error("Cannot open the collision OBJ");
        while(std::getline(f,line)) {
            if(line.size()<3 || line[1]!=' ')continue;
            std::istringstream row(line.substr(2));
            if(line[0]=='o') {meshes.emplace_back();row>>meshes.back().name;}
            else if(line[0]=='v') {
                Vec v;row>>v.x>>v.y>>v.z;auto& m=meshes.at(meshes.size()-1);
                m.vertices.push_back(int(verts.size()));verts.push_back(v);m.lo=low(m.lo,v);m.hi=high(m.hi,v);
            } else if(line[0]=='f') {
                int a,b,c;row>>a>>b>>c;
                if(!row || a<1 || b<1 || c<1 || size_t(std::max({a,b,c}))>verts.size())
                    throw std::runtime_error("Expected triangulated OBJ with 1-based vertex indices");
                meshes.back().tris.push_back({a-1,b-1,c-1});
            }
        }
        for(auto& m:meshes) {
            // Exported OBJ vertices are already in world space. For a tall
            // prop reconstruct its stem centre from its lowest geometry.
            Vec root=(m.lo+m.hi)*.5f; unsigned count=0;Vec total{};
            for(int i:m.vertices) if(verts[i].z<m.lo.z+.26f) {total=total+verts[i];++count;}
            if(count)root=total*(1.f/count);
            m.shape.origin=root;m.shape.lo=m.lo-root;m.shape.hi=m.hi-root;
        }
    }
    const std::string& name(uintptr_t id) const {static std::string portal="roof";return id?meshes[id-1].name:portal;}
    int nearby(Vec centre,float r,Vec forward,Body* out,int cap) override {
        ++queries; int n=0; Vec delta{r,r,r};
        for(size_t i=0;i<meshes.size() && n<cap;i++) {
            auto& m=meshes[i];
            if(overlap(centre-delta,centre+delta,m.lo,m.hi) && bodyFrom(m.shape,forward,i+1,false,{},out[n]))++n;
        }
        return n;
    }
    bool trace(Vec a,Vec b,Hit& hit) override {
        ++rays;bool found=false;Vec d=b-a,l=low(a,b),h=high(a,b);float nearest=1.0001f;
        for(size_t i=0;i<meshes.size();i++) {
            auto& m=meshes[i];if(!overlap(l,h,m.lo,m.hi))continue;
            for(auto face:m.tris) {
                Vec v=verts[face.a],e=verts[face.b]-v,g=verts[face.c]-v,q=cross(d,g);
                float det=dot(e,q);if(std::abs(det)<.000001f)continue;
                Vec diff=a-v;float u=dot(diff,q)/det;if(u<0 || u>1)continue;
                Vec j=cross(diff,e);float w=dot(d,j)/det;if(w<0 || u+w>1)continue;
                float t=dot(g,j)/det;if(t<.00001f || t>=nearest || t>1)continue;
                nearest=t;Vec normal=cross(e,g);Body unused;
                hit={a+d*t,normal*(1.f/length(normal)),i+1,bodyFrom(m.shape,{1,0,0},i+1,false,{},unused),overheadPanel(m.shape.hi-m.shape.lo)};
                found=true;
            }
        }
        return found;
    }
};
int main(int argc,char** argv) {
    if(argc!=2){std::cerr<<"scene_runner path/to/LAe2_collision.obj < routes.txt\n";return 2;}
    Scene world(argv[1]);std::string line;
    while(std::getline(std::cin,line)) {
        if(line.empty() || line[0]=='#')continue;
        std::istringstream row(line);std::string mode;row>>mode;
        if(mode=="camera") {
            Vec from,to;
            row>>from.x>>from.y>>from.z>>to.x>>to.y>>to.z;
            if(!row) return 2;
            Hit hit{};
            auto cast=[&](Vec a,Vec b,Vec& point) {
                if(!world.trace(a,b,hit)) return false;
                point=hit.point; return true;
            };
            auto eye=camera::limitTravel(from,to,.24f,cast);
            std::printf("%s %.6f %.6f\n",hit.id ? world.name(hit.id).c_str() : "open",
                        length(eye-from),hit.id ? length(hit.point-from) : length(to-from));
            continue;
        }
        if(mode=="ray") {
            Vec a,b;row>>a.x>>a.y>>a.z>>b.x>>b.y>>b.z;Hit h;
            if(world.trace(a,b,h))std::cout<<world.name(h.id)<<" "<<h.point.x<<" "<<h.point.y<<" "<<h.point.z<<"\n";
            else std::cout<<"open\n";
            continue;
        }
        if(mode!="route")return 2;
        std::string label;Vec a,b;float kmh,fps;row>>label>>a.x>>a.y>>a.z>>b.x>>b.y>>b.z>>kmh>>fps;
        if(!row || kmh<=0 || fps<=0 || length(b-a)<.01f)return 2;
        Vec dir=(b-a)*(1.f/length(b-a)),right{dir.y,-dir.x,0};
        unsigned frames=unsigned(std::ceil(length(b-a)*fps*3.6f/kmh));Detector detector;world.rays=world.queries=0;
        std::array<int,3> counts{};
        for(unsigned i=0;i<=frames;i++) {
            float dist=std::min(length(b-a),kmh/3.6f*i/fps);
            auto events=detector.update(world,{a+dir*dist,dir*(kmh/3.6f),right,uint32_t(1000+i*1000./fps),1});
            for(int j=0;j<events.count;j++) {
                auto& e=events.list[j];auto k=int(e.kind);++counts[k];
                std::printf("hit %s %s %.2f %.4f %.3f %s %.3f %.3f %.4f %.4f %d\n",label.c_str(),
                            k==0?"small":k==1?"large":e.family==Family::Exit?"exit":e.family==Family::Bridge?"bridge":"tunnel",dist,e.gain,e.pan,world.name(e.id).c_str(),
                            e.motion.distance,e.motion.relativeSpeed,e.motion.secondsToPass,e.motion.headingPan,int(e.family));
            }
        }
        std::printf("total %s %d %d %d %u %u\n",label.c_str(),counts[0],counts[1],counts[2],world.rays,world.queries);
    }
}
