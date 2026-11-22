#include "../source/mixer.hpp"
#include "../source/wind.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
using namespace whoosh;
using namespace whoosh::audio;
namespace fs=std::filesystem;
static double energy(const std::array<int16_t,blockFrames*2>& pcm,unsigned side) {
    double total=0;
    for(size_t i=side;i<pcm.size();i+=2)total+=double(pcm[i])*pcm[i];
    return total;
}
int main(int argc,char** argv) {
    if(argc!=2 && argc!=4) {std::cerr<<"mixer_check sample_bank [events.txt output.s16le]\n";return 2;}
    Bank bank;
    for(auto& file:fs::directory_iterator(argv[1])) {
        Sample sample;
        FILE* f=std::fopen(file.path().string().c_str(),"rb");
        assert(f && readPcm16(f,sample));std::fclose(f);
        sample.number=std::stoi(file.path().stem().string().substr(7));
        bank[0].push_back(std::move(sample));
    }
    assert(bank[0].size()==23);
    std::array<int16_t,blockFrames*2> block{};
    const auto it=std::find_if(bank[0].begin(),bank[0].end(),[](const Sample& s){return s.number==13;});
    assert(it!=bank[0].end());
    Bank only;only[0].push_back(*it);
    Mixer reference(std::move(only),7),all(bank,7);
    Event panel{Kind::Large,.75f,.2f,5};panel.panel=true;
    reference.accept(panel);all.accept(panel);
    assert(reference.active());
    do {
        decltype(block) a,b;reference.render(a,1);all.render(b,1);assert(a==b);
    } while(reference.active());
    assert(!all.active());
    Bank tone;
    Sample steady{48000,1,std::vector<int16_t>(24000,12000)};
    steady.number=13;steady.crest=.16f;tone[0].push_back(steady);
    Mixer passing(tone,4);
    Event moving{Kind::Small,.65f,0,1,{2.f,30.f,.16f,1.f}};moving.panel=true;
    passing.accept(moving);passing.render(block,1);assert(energy(block,1)>energy(block,0)*6);
    for(int i=0;i<40;i++)passing.render(block,1);
    assert(energy(block,0)>energy(block,1)*6);
    Mixer summed(tone,9);
    panel.pan=0;panel.gain=1;
    for(int i=0;i<8;i++)summed.accept(panel);
    summed.render(block,1);summed.render(block,1);
    assert(block[200]>25000 && block[200]<26900 && block[200]==block[201]);
    Mixer mix(bank,23);
    for(int i=0;i<12;i++)mix.accept({Kind::Tunnel,1,0,0});
    for(int b=0;b<35;b++) {
        mix.render(block,1);
        for(auto sample:block)assert(sample>-32768 && sample<=32767);
    }
    mix.clear();mix.render(block,1);assert(!energy(block,0) && !energy(block,1));
    for(auto& file:fs::directory_iterator(fs::path(argv[1]).parent_path()/"Wind")) {
        FILE* f=std::fopen(file.path().string().c_str(),"rb");Sample s;
        assert(f && readPcm16(f,s));std::fclose(f);
        s.number=std::stoi(file.path().stem().string().substr(5));bank[1].push_back(std::move(s));
    }
    assert(bank[1].size()==5);
    bool preferred;
    assert(windNumber(L"WIND 1.wav",preferred)==1 && preferred);
    assert(windNumber(L"Wind 05.WAV",preferred)==5 && !preferred);
    assert(windNumber(L"wind 1x.wav",preferred)==0);
    assert(windWeight(0,.22f)>.9f && windWeight(1,.50f)>.95f && windWeight(2,1)==1);
    Bank weatherOnly; for(auto& s:bank[1])if(s.number==4)weatherOnly[1].push_back(s);
    Mixer weather(std::move(weatherOnly),8);weather.wind(1,1);
    for(int i=0;i<200;i++){weather.render(block,1);assert(!energy(block,0) && !energy(block,1));}
    Mixer wind(bank,8);
    assert(!wind.active());
    auto runWind=[&](float speed) {
        wind.wind(speed,.4f);double sum=0;
        for(unsigned i=0;i<400;++i) {wind.render(block,1);if(i>=200)sum+=energy(block,0)+energy(block,1);}
        return sum;
    };
    double gentle=runWind(.15f);assert(gentle>0 && runWind(1)>gentle*4);
    runWind(0);assert(!wind.active());
    Mixer stitches(bank,6);
    for(int family=1;family<=7;++family) {
        bool three=family==1 || family==2 || family==5 || family==6;
        int variants=(family<=4)?6:3;
        for(int band=0;band<(three?3:2);++band)for(int v=0;v<variants;++v) {
            Event e{};e.family=Family(family);e.gain=.9f;e.intensity=(band+.5f)/(three?3:2);
            stitches.accept(e);double power=0;int blocks=0;
            while(stitches.active()) {stitches.render(block,1);power+=energy(block,0);assert(++blocks<650);}
            assert(power>0);
        }
    }
    if(argc==4) {
        struct Cue {double seconds;Event event;};
        std::vector<Cue> cues;std::ifstream events(argv[2]);Cue cue;unsigned kind,family;
        auto& motion=cue.event.motion;
        while(events>>cue.seconds>>kind>>cue.event.gain>>cue.event.pan
                    >>motion.distance>>motion.relativeSpeed>>motion.secondsToPass>>motion.headingPan>>family) {
            assert(kind<3);cue.event.kind=Kind(kind);assert(family>=1 && family<=7);cue.event.family=Family(family);cues.push_back(cue);
        }
        assert(!cues.empty());
        Mixer preview(std::move(bank),1979);
        std::ofstream pcm(argv[3],std::ios::binary);assert(pcm);
        size_t next=0;
        for(unsigned i=0;i*double(blockFrames)/outputRate<cues.back().seconds+1.18;i++) {
            while(next<cues.size() && cues[next].seconds<=i*double(blockFrames)/outputRate)
                preview.accept(cues[next++].event);
            preview.render(block,.8f);
            pcm.write(reinterpret_cast<const char*>(block.data()),sizeof(block));
        }
    }
    std::cout<<"23 WAVs decoded and rendered; stereo, duration, overlap and stop checks passed.\n";
}
