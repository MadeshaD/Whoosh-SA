#pragma once
#include "detector.hpp"
#include <cstdio>
#include <utility>
#include <vector>

namespace whoosh::audio {
constexpr unsigned outputRate=48000,blockFrames=384;
struct Sample { unsigned rate{},channels{}; std::vector<int16_t> data; float crest=.07f,trim=1.f; int number{}; };
using Bank=std::array<std::vector<Sample>,3>;
bool readPcm16(FILE* file,Sample& sample);
class Mixer {
public:
    explicit Mixer(Bank samples,unsigned seed);
    void accept(Event event);
    bool active() const;
    void nitrous(float power);
    void clear() { nitroPower=loopGain=0; loopPosition=0; shots={}; voices={}; windSpeed=targetSpeed=0; phase={}; }
    void wind(float ratio,float volume) { targetSpeed=unitRange(ratio); windGain=unitRange(volume); }
    void render(std::array<int16_t,blockFrames*2>& pcm,float gain);
private:
    struct Voice {
        const Sample* sample{};
        double pos{},step{};
        Event event{};
    };
    void mixNitrous(std::array<float,blockFrames*2>& output);
    enum : unsigned { sustain=1, release=2, attack=3 };
    std::array<const Sample*,4> nitroSamples{};
    struct NitroShot { unsigned sample{}; double position{}; float gain{}; bool fading{}; };
    std::array<NitroShot,4> shots{};
    float nitroPower{},loopGain{};
    double loopPosition{};
    float nitroFrame(unsigned sample,double& position,bool repeat) const;
    Bank bank;
    void stitch(Event e);
    void mixWind(std::array<float,blockFrames*2>& output);
    std::array<const Sample*,24> numbered{};
    std::array<const Sample*,5> windSamples{};
    std::array<double,5> phase{};
    std::array<unsigned,8> cycles{};
    float targetSpeed{},windSpeed{},windGain{};
    std::array<Voice,64> voices{};
    unsigned seed;
};
}
