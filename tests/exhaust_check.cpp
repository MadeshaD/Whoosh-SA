#include "../source/exhaust.hpp"
#include <cassert>

using namespace whoosh::visuals;
int main() {
    for(int i=0;i<4;++i){
        auto weights=exhaustKeyWeights(i/3.f);
        for(int j=0;j<4;++j)assert(std::abs(weights[j]-float(i==j))<1e-5f);
    }
    // Straight-line case with 30 m/s of vehicle velocity inherited at emission.
    for(unsigned type:{0u,1u,2u}){
        auto& e=exhaustEmitters[type];
        ExhaustPuff p{};
        p.velocity={0,30-e.speed,0};
        p.age=e.life;
        auto end=p.position(e.gravity);
        float separation=30*p.age-end.y;
        assert(std::abs(separation-(type==2 ? 1.75f : .675f))<1e-5f);
        assert(end.z>0);
    }
    // World quad projected through a 90-degree horizontal FOV, with the 0.2 m view bias.
    auto widthAt=[](float depth){return 960.f*2*lampHalfSizeMetres/(depth-.2f);};
    assert(std::abs(widthAt(5.2f)-2*widthAt(10.2f))<.001f);
    assert(std::abs(widthAt(10.2f)-2*widthAt(20.2f))<.001f);
}
