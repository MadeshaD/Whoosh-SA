#include "../source/mixer.hpp"
#include <cassert>
#include <filesystem>

using namespace whoosh::audio;
int main(int argc,char** argv) {
    assert(argc==2 || argc==3);
    Bank bank;
    for(int n=1;n<=3;++n) {
        auto path=std::filesystem::path(argv[1])/("Nitrous 00 "+std::to_string(n)+".wav");
        auto file=std::fopen(path.string().c_str(),"rb");
        Sample s;
        s.number=n;
        assert(file && readPcm16(file,s));
        std::fclose(file);
        bank[2].push_back(std::move(s));
    }
    // Isolate each recorded layer and prove that both reach the same output block.
    auto layerPower=[&](unsigned sample,bool onRelease){
        Bank solo=bank;
        for(auto& s:solo[2])if(s.number!=int(sample))std::fill(s.data.begin(),s.data.end(),0);
        Mixer mixer(std::move(solo),1);
        std::array<int16_t,blockFrames*2> pcm{};
        mixer.nitrous(1);
        for(int i=0;i<25;++i)mixer.render(pcm,1);
        if(onRelease){mixer.nitrous(.5f);for(int i=0;i<30;++i)mixer.render(pcm,1);}
        double sum=0;
        for(auto v:pcm)sum+=double(v)*v;
        return sum/pcm.size();
    };
    assert(layerPower(3,false)>100 && layerPower(1,false)>100);
    assert(layerPower(2,true)>100 && layerPower(1,true)>100);
    Mixer mixer(std::move(bank),1);
    auto preview=argc==3 ? std::fopen(argv[2],"wb") : nullptr;
    if(argc==3)assert(preview);
    std::array<int16_t,blockFrames*2> pcm{};
    auto drive=[&](float power,int blocks){
        double sum=0;
        for(int i=0;i<blocks;++i){
            mixer.nitrous(power);
            mixer.render(pcm,1);
            if(preview)std::fwrite(pcm.data(),sizeof(int16_t),pcm.size(),preview);
            if(i>=blocks/2)for(auto v:pcm)sum+=double(v)*v;
        }
        return sum/(pcm.size()*(blocks-blocks/2));
    };
    double boost=drive(1,375);
    double coast=drive(.5f,200);
    assert(coast>100 && boost>coast*9);
    assert(drive(1,125)>coast*9);
    drive(0,150);
    assert(!mixer.active());
    for(int i=0;i<12;++i)drive(i%2?0.f:1.f,2);
    drive(0,180);
    assert(!mixer.active());
    if(preview)std::fclose(preview);
    mixer.nitrous(1);
    mixer.render(pcm,1);
    mixer.clear();
    mixer.render(pcm,1);
    assert(std::all_of(pcm.begin(),pcm.end(),[](int16_t v){return !v;}));
}
