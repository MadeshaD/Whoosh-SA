#include "mixer.hpp"
#include <cstring>
#include "recipes.hpp"
#include "wind.hpp"

namespace whoosh::audio {
namespace {
uint16_t u16(const unsigned char* b) { return b[0]|(uint16_t(b[1])<<8); }
uint32_t u32(const unsigned char* b) { return u16(b)|(uint32_t(u16(b+2))<<16); }
}
bool readPcm16(FILE* file,Sample& sample) {
    if(!file || std::fseek(file,0,SEEK_END))return false;
    long size=std::ftell(file);
    if(size<44 || size>8*1024*1024 || std::fseek(file,0,SEEK_SET))return false;
    std::vector<unsigned char> data(size);
    if(std::fread(data.data(),1,data.size(),file)!=data.size() ||
       std::memcmp(data.data(),"RIFF",4) || std::memcmp(data.data()+8,"WAVE",4))return false;
    size_t end=std::min<uint64_t>(data.size(),8ULL+u32(data.data()+4));
    const unsigned char* pcm=nullptr; size_t bytes=0;
    for(size_t offset=12;offset+8<=end;) {
        auto* header=data.data()+offset; size_t n=u32(header+4); offset+=8;
        if(n>end-offset)return false;
        if(!std::memcmp(header,"fmt ",4) && n>=16) {
            auto* f=data.data()+offset;
            if(u16(f)!=1 || u16(f+14)!=16)return false;
            sample.channels=u16(f+2); sample.rate=u32(f+4);
            if(sample.channels<1 || sample.channels>2 || sample.rate<8000 || sample.rate>96000 ||
               u16(f+12)!=sample.channels*2)return false;
        } else if(!std::memcmp(header,"data",4)) { pcm=data.data()+offset; bytes=n; }
        offset+=n+(n&1);
    }
    if(!pcm || !sample.channels || bytes%(2*sample.channels) || bytes<4*sample.channels)return false;
    sample.data.resize(bytes/2); std::memcpy(sample.data.data(),pcm,bytes);
    // Locate the first strong part of the envelope in 8 ms windows. Preserve
    // the WAV, and store its acoustic timing and headroom separately.
    const size_t frames=sample.data.size()/sample.channels,window=std::max(1u,sample.rate/125);
    double loudest=0;std::vector<double> envelope;
    for(size_t start=0;start<frames;start+=window) {
        size_t end=std::min(frames,start+window);double power=0;
        for(size_t i=start*sample.channels;i<end*sample.channels;i++) {
            double v=sample.data[i]/32768.;power+=v*v;
        }
        double rms=std::sqrt(power/((end-start)*sample.channels));
        envelope.push_back(rms);loudest=std::max(loudest,rms);
    }
    size_t first=0;while(first+1<envelope.size() && envelope[first]<loudest*.78)++first;
    sample.crest=float((first+.5)*window/sample.rate);
    sample.trim=float(std::min(1.,.24/std::max(.001,loudest)));
    return true;
}
Mixer::Mixer(Bank samples,unsigned randomSeed):bank(std::move(samples)),seed(randomSeed?randomSeed:1) {
    for(unsigned group=0;group<bank.size();++group) {
        for(auto& s:bank[group]) {
            if(group==0 && s.number>0 && s.number<24)numbered[s.number]=&s;
            if(group==2 && s.number>=1 && s.number<=3)nitroSamples[s.number]=&s;
            if(group==1 && s.number>0 && s.number<=5)windSamples[s.number-1]=&s;
        }
    }
}
bool Mixer::active() const {
    if(nitroPower>0 || loopGain>0)return true;
    for(const auto& shot:shots)if(shot.sample)return true;
    for(auto& v:voices)if(v.sample)return true;
    return windGain>0 && !bank[1].empty() && (targetSpeed>0 || windSpeed>.0001f);
}
void Mixer::stitch(Event e) {
    struct Group {unsigned first,bands,variants;};
    constexpr Group groups[]={{0,0,0},{9,3,6},{57,3,6},{33,2,6},{45,2,6},{0,3,3},{75,3,3},{84,2,3}};
    unsigned family=static_cast<unsigned>(e.family);
    const auto g=groups[family];
    unsigned band=std::min(g.bands-1,unsigned(unitRange(e.intensity)*g.bands));
    unsigned id=g.first+g.variants*band+cycles[family]++%g.variants;
    auto variation=[&](int range) {
        seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;
        return int(seed%unsigned(1+std::abs(range)));
    };
    for(unsigned i=recipeOffsets[id];i<recipeOffsets[id+1];++i) {
        const auto& l=layers[i];
        const Sample* s=numbered[l.wav];
        if(!s)continue;
        auto slot=std::find_if(voices.begin(),voices.end(),[](const Voice& v){return !v.sample;});
        if(slot==voices.end())break;
        Event layer=e;
        float pitch=std::max(.1f,(l.pitch-variation(l.pitchSpread))/4096.f);
        double step=s->rate*double(pitch)/outputRate;
        double wait=std::max(0.f,e.motion.secondsToPass-.105f)+l.offset*.001;
        layer.motion.secondsToPass-=float(wait);
        layer.pan=std::clamp(e.pan+float(std::sin(l.az*(6.283185307/65536.)))*.75f*(1-std::abs(e.pan)),-.98f,.98f);
        layer.gain*=std::max(0,l.volume-variation(l.volumeSpread))/32767.f*std::sqrt(s->trim);
        *slot={s,-wait*outputRate*step,step,layer};
    }
}
void Mixer::accept(Event e) {
    if(e.panel && numbered[13]) {
        const auto& sample=*numbered[13];
        auto slot=std::find_if(voices.begin(),voices.end(),[](const Voice& v){return !v.sample;});
        if(slot==voices.end())return;
        const double wait=std::max(0.,double(e.motion.secondsToPass)-sample.crest-.018);
        e.motion.secondsToPass=std::max(0.f,e.motion.secondsToPass-float(wait)-.018f);
        e.gain*=sample.trim;
        *slot={&sample,-wait*sample.rate,double(sample.rate)/outputRate,e};
        return;
    }
    stitch(e);
}
namespace {
std::array<float,2> channelLevels(const Event& e,double seconds) {
    float pan=e.pan; const auto& m=e.motion;
    if(m.relativeSpeed>0 && m.distance>.01f) {
        float ahead=(m.secondsToPass-static_cast<float>(seconds))*m.relativeSpeed;
        pan=std::clamp((e.pan*m.distance+m.headingPan*ahead)/std::hypot(m.distance,ahead),-.95f,.95f);
    }
    return {e.gain*std::sqrt((1.f-pan)*.5f),e.gain*std::sqrt((1.f+pan)*.5f)};
}
}
void Mixer::mixWind(std::array<float,blockFrames*2>& out) {
    if(windGain<=0 || bank[1].empty() || (windSpeed==0 && targetSpeed==0))return;
    const float start=windSpeed;
    windSpeed+=(targetSpeed-windSpeed)*(targetSpeed>windSpeed?.065f:.038f);
    if(!targetSpeed && windSpeed<.0001f)windSpeed=0;
    for(unsigned layer=0;layer<windSamples.size();++layer) {
        const Sample* s=windSamples[layer];
        if(!s || layer==3)continue;
        const size_t n=s->data.size()/s->channels;
        const size_t overlap=std::min(n/4,size_t(.030*s->rate));
        const double period=double(n-overlap);
        auto read=[&](double p,unsigned chan) {
            auto interp=[&](double x) {
                size_t j=size_t(x),k=std::min(j+1,n-1);
                unsigned c=chan%s->channels;
                float a=s->data[j*s->channels+c],b=s->data[k*s->channels+c];
                return (a+(b-a)*float(x-j))/32768.f;
            };
            p+=overlap;
            float value=interp(p);
            if(overlap && p>=period) {
                float blend=float((p-period)/overlap);
                value+=(interp(p-period)-value)*blend;
            }
            return value;
        };
        for(unsigned i=0;i<blockFrames;++i) {
            float speed=start+(windSpeed-start)*(i+1)/blockFrames;
            float weight=windWeight(layer,speed);
            double p=phase[layer];
            if(weight>0) {
                float left=read(p,0);
                double other=p+std::min(period*.4,s->rate*.137);
                float right=read(other>=period?other-period:other,1);
                float mid=(left+right)*.5f,side=(left-right)*(.12f+.78f*speed)*.5f;
                float gain=weight*speed*windGain;
                out[2*i]+=gain*(mid+side);out[2*i+1]+=gain*(mid-side);
            }
            p+=double(s->rate)/outputRate;
            phase[layer]=p>=period?std::fmod(p,period):p;
        }
    }
}
void Mixer::nitrous(float power) {
    if(!nitroSamples[1] || !nitroSamples[2] || !nitroSamples[3])return;
    bool wasBoosting=nitroPower>.5f;
    bool boosting=power>.5f;
    nitroPower=power;
    if(wasBoosting==boosting)return;
    // Keep previously audible voices alive during a reversal of the throttle.
    for(auto& shot:shots)if(shot.sample)shot.fading=true;
    auto slot=std::min_element(shots.begin(),shots.end(),[](const auto& a,const auto& b){
        return (a.sample?a.gain:-1.f)<(b.sample?b.gain:-1.f);
    });
    *slot={boosting?attack:release,0,1,false};
}
float Mixer::nitroFrame(unsigned index,double& pos,bool repeat) const {
    const auto& s=*nitroSamples[index];
    size_t frames=s.data.size()/s.channels;
    size_t j=size_t(pos);
    size_t next=j+1<frames ? j+1 : (repeat?0:j);
    float a=s.data[j*s.channels],b=s.data[next*s.channels];
    float value=(a+(b-a)*float(pos-j))/32768.f;
    pos+=double(s.rate)/outputRate;
    if(repeat && pos>=frames)pos-=frames;
    return value;
}
void Mixer::mixNitrous(std::array<float,blockFrames*2>& output) {
    if(!nitroSamples[sustain])return;
    float throttle=unitRange(2*nitroPower-1);
    float target=nitroPower>0 ? .062f+.295f*throttle : 0.f;
    float step=target>loopGain ? .00016f : .000085f;
    for(unsigned i=0;i<blockFrames;++i) {
        loopGain+=std::clamp(target-loopGain,-step,step);
        float value=0;
        if(loopGain>0)value=loopGain*nitroFrame(sustain,loopPosition,true);
        else loopPosition=0;
        for(auto& shot:shots) {
            if(!shot.sample)continue;
            auto& s=*nitroSamples[shot.sample];
            double remaining=s.data.size()/s.channels-shot.position;
            float edge=float(std::min({1.,shot.position/(s.rate*.006),remaining/(s.rate*.012)}));
            float weight=shot.sample==attack ? .40f : .31f;
            value+=nitroFrame(shot.sample,shot.position,false)*weight*edge*shot.gain*.4f;
            if(shot.fading)shot.gain-=1.f/(.15f*outputRate);
            if(shot.gain<=0 || shot.position>=s.data.size()/s.channels)shot={};
        }
        output[2*i]+=value;
        output[2*i+1]+=value;
    }
}
void Mixer::render(std::array<int16_t,blockFrames*2>& pcm,float gain) {
    std::array<float,blockFrames*2> mix{};
    for(auto& v:voices) {
        if(!v.sample)continue;
        const auto& s=*v.sample; size_t frames=s.data.size()/s.channels;
        // Interpolate stereo gains between block endpoints. The apparent
        // source moves from ahead of the car to behind it during playback.
        double seconds=std::max(0.,v.pos/(v.step*outputRate));
        auto levels=channelLevels(v.event,seconds);
        auto end=channelLevels(v.event,seconds+double(blockFrames)/outputRate);
        std::array<float,2> delta{(end[0]-levels[0])/blockFrames,(end[1]-levels[1])/blockFrames};
        for(unsigned i=0;i<blockFrames;i++) {
            if(v.pos<0) {v.pos+=v.step;continue;}
            size_t n=static_cast<size_t>(v.pos);
            if(n>=frames) {v.sample=nullptr;break;}
            size_t m=std::min(n+1,frames-1);
            float f=static_cast<float>(v.pos-n);
            float fade=std::min(unitRange(float(v.pos)/(s.rate*.009f)),
                                unitRange(float(frames-v.pos)/std::min(frames*.26f,s.rate*.046f)));
            fade=fade*fade*(3.f-2.f*fade);
            for(unsigned ch=0;ch<2;ch++) {
                unsigned c=s.channels==1?0:ch;
                float a=s.data[n*s.channels+c],z=s.data[m*s.channels+c];
                mix[2*i+ch]+=(a+(z-a)*f)*levels[ch]*fade/32768.f;
                levels[ch]+=delta[ch];
            }
            v.pos+=v.step;
        }
    }
    mixWind(mix);
    mixNitrous(mix);
    for(unsigned i=0;i<blockFrames;i++) {
        float left=mix[2*i]*gain,right=mix[2*i+1]*gain;
        float peak=std::max(std::abs(left),std::abs(right));
        // Linked soft knee leaves stereo direction intact during overlap.
        float factor=1;
        if(peak>.66f)factor=(.66f+.16f*(-std::expm1(-(peak-.66f)/.16f)))/peak;
        pcm[2*i]=static_cast<int16_t>(32767*left*factor);
        pcm[2*i+1]=static_cast<int16_t>(32767*right*factor);
    }
}
}
