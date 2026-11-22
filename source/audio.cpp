#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>
#include <xaudio2.h>
#include "audio.hpp"
#include "mixer.hpp"
#include "wind.hpp"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>

namespace whoosh::audio {
namespace {
constexpr unsigned numBlocks=3;
using File=std::unique_ptr<FILE,decltype(&std::fclose)>;
std::atomic<float> windGain{};
std::atomic<float> speedRatio{};
std::atomic<unsigned> whooshFiles{},windFiles{};
std::atomic<const char*> backend{"idle"};
Bank loadBank(const std::wstring& root) {
    Bank bank;
    for(unsigned i=0;i<bank.size();++i) {
        auto dir=root+(i==1 ? L"\\Whoosh SFX\\Wind\\" : L"\\Whoosh SFX\\Whoosh\\");
        WIN32_FIND_DATAW data{}; HANDLE find=FindFirstFileW((dir+L"*.wav").c_str(),&data);
        if(find==INVALID_HANDLE_VALUE)continue;
        std::vector<std::wstring> files;
        do {
            if(!(data.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY)) files.emplace_back(data.cFileName);
        }while(FindNextFileW(find,&data));
        FindClose(find); std::sort(files.begin(),files.end());
        // Decode the preferred file first. If it is damaged, try the older alias.
        if(i==1)std::stable_sort(files.begin(),files.end(),[](const auto& a,const auto& b) {
            return a.size()<b.size();
        });
        std::array<bool,6> loaded{};
        for(auto& name:files) {
            Sample s;
            int number{};
            if(i==1) {
                bool canonical;
                number=windNumber(name,canonical);
                if(!number || number==4 || loaded[number])continue;
                s.number=number;
            } else {
                int parsed=0;
                if(std::swscanf(name.c_str(),L"Whoosh %d%n",&number,&parsed)!=1 ||
                   _wcsicmp(name.c_str()+parsed,L".wav") || number<1 || number>23)continue;
                s.number=number;
            }
            File file(_wfopen((dir+name).c_str(),L"rb"),std::fclose);
            if(readPcm16(file.get(),s)) {
                if(i==1)loaded[number]=true;
                bank[i].push_back(std::move(s));
            }
        }
    }
    whooshFiles=bank[0].size();windFiles=bank[1].size();
    return bank;
}
struct Command { Event event; unsigned epoch; DWORD when; };
constexpr unsigned queueSize=128;
std::array<Command,queueSize> queue{};
std::atomic<unsigned> writer{},reader{},epoch{};
std::atomic<float> master{};
HANDLE wake{};
std::wstring path;
// Single consumer shared by the primary device and the compatibility path.
void drain(Mixer& mixer,unsigned current) {
    mixer.wind(speedRatio.load(std::memory_order_relaxed),windGain.load(std::memory_order_relaxed));
    auto r=reader.load(std::memory_order_relaxed),end=writer.load(std::memory_order_acquire);
    DWORD now=GetTickCount();
    while(r!=end) {
        auto cmd=queue[r++%queueSize];
        if(cmd.epoch==current && now-cmd.when<200)mixer.accept(cmd.event);
    }
    reader.store(r,std::memory_order_release);
}
class NativeVoice final : public IXAudio2VoiceCallback,public IXAudio2EngineCallback {
public:
    explicit NativeVoice(HANDLE signal):signal(signal) {}
    ~NativeVoice() {
        if(engine)engine->StopEngine();
        if(registered)engine->UnregisterForCallbacks(this);
        if(voice)voice->DestroyVoice();
        if(masterVoice)masterVoice->DestroyVoice();
        if(engine)engine->Release();
        if(dll)FreeLibrary(dll);
    }
    bool open() {
        dll=LoadLibraryExW(L"xaudio2_9.dll",nullptr,LOAD_LIBRARY_SEARCH_SYSTEM32);
        if(!dll)dll=LoadLibraryExW(L"xaudio2_8.dll",nullptr,LOAD_LIBRARY_SEARCH_SYSTEM32);
        if(!dll)return false;
        using Factory=HRESULT(WINAPI*)(IXAudio2**,UINT32,XAUDIO2_PROCESSOR);
        auto raw=GetProcAddress(dll,"XAudio2Create");Factory create{};
        static_assert(sizeof(create)==sizeof(raw));std::memcpy(&create,&raw,sizeof(raw));
        if(!create || FAILED(create(&engine,0,XAUDIO2_DEFAULT_PROCESSOR)))return false;
        if(FAILED(engine->RegisterForCallbacks(this)))return false;
        registered=true;
        if(FAILED(engine->CreateMasteringVoice(&masterVoice)))return false;
        WAVEFORMATEX fmt{WAVE_FORMAT_PCM,2,outputRate,outputRate*4,4,16,0};
        if(FAILED(engine->CreateSourceVoice(&voice,&fmt,0,1.f,this)))return false;
        return SUCCEEDED(voice->Start());
    }
    bool fill(Mixer& mixer,float gain) {
        if(error.load())return false;
        XAUDIO2_VOICE_STATE state{};voice->GetState(&state,XAUDIO2_VOICE_NOSAMPLESPLAYED);
        for(unsigned queued=state.BuffersQueued;queued<numBlocks;++queued) {
            auto& block=pcm[next];
            if(mixer.active())mixer.render(block,gain);else block.fill(0);
            XAUDIO2_BUFFER buffer{};buffer.AudioBytes=sizeof(block);buffer.pAudioData=reinterpret_cast<BYTE*>(block.data());
            if(FAILED(voice->SubmitSourceBuffer(&buffer)))return false;
            next=(next+1)%numBlocks;
        }
        return true;
    }
    void STDMETHODCALLTYPE OnBufferEnd(void*) override {SetEvent(signal);}
    void STDMETHODCALLTYPE OnCriticalError(HRESULT) override {error=true;SetEvent(signal);}
    void STDMETHODCALLTYPE OnVoiceError(void*,HRESULT e) override {OnCriticalError(e);}
    void STDMETHODCALLTYPE OnVoiceProcessingPassStart(UINT32) override {}
    void STDMETHODCALLTYPE OnVoiceProcessingPassEnd() override {}
    void STDMETHODCALLTYPE OnProcessingPassStart() override {}
    void STDMETHODCALLTYPE OnProcessingPassEnd() override {}
    void STDMETHODCALLTYPE OnBufferStart(void*) override {}
    void STDMETHODCALLTYPE OnLoopEnd(void*) override {}
    void STDMETHODCALLTYPE OnStreamEnd() override {}
private:
    HANDLE signal;
    HMODULE dll{};
    IXAudio2* engine{};IXAudio2MasteringVoice* masterVoice{};IXAudio2SourceVoice* voice{};
    bool registered{};
    unsigned next{};std::atomic<bool> error{};
    std::array<std::array<int16_t,blockFrames*2>,numBlocks> pcm{};
};
struct Block { WAVEHDR header{}; std::array<int16_t,blockFrames*2> pcm{}; bool queued{}; };
class Output {
public:
    explicit Output(const std::wstring& root):mixer(loadBank(root),GetTickCount()|1u) {}
    ~Output() { close(); if(done)CloseHandle(done); }
    void loop();
private:
    HANDLE done=CreateEventW(nullptr,FALSE,FALSE,nullptr);
    HWAVEOUT device{};
    Mixer mixer;
    std::array<Block,numBlocks> blocks{};
    unsigned nextBlock{};
    DWORD retryAfter{};
    bool failed{};
    bool open();
    void close();
    void cancel();
};
bool Output::open() {
    if(device)return true;
    WAVEFORMATEX fmt{WAVE_FORMAT_PCM,2,outputRate,outputRate*4,4,16,0};
    if(waveOutOpen(&device,WAVE_MAPPER,&fmt,reinterpret_cast<DWORD_PTR>(done),0,CALLBACK_EVENT)!=MMSYSERR_NOERROR) {
        device=nullptr; return false;
    }
    for(auto& b:blocks) {
        b.header={}; b.header.lpData=reinterpret_cast<char*>(b.pcm.data());
        b.header.dwBufferLength=sizeof(b.pcm);
        if(waveOutPrepareHeader(device,&b.header,sizeof(b.header))!=MMSYSERR_NOERROR) { close(); return false; }
    }
    return true;
}
void Output::cancel() {
    mixer.clear(); if(device)waveOutReset(device);
    for(auto& b:blocks)b.queued=false;
    nextBlock=0;
}
void Output::close() {
    if(!device)return;
    cancel();
    for(auto& b:blocks)if(b.header.dwFlags&WHDR_PREPARED)
        waveOutUnprepareHeader(device,&b.header,sizeof(b.header));
    waveOutClose(device);device=nullptr;
}
void Output::loop() {
    if(!done)return;
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_ABOVE_NORMAL);
    HANDLE waiters[]={wake,done}; unsigned previous=epoch.load();
    HRESULT com=CoInitializeEx(nullptr,COINIT_MULTITHREADED);
    {
        NativeVoice primary(done);
        if(primary.open()) {
            backend="XAudio2";
            for(;;) {
                unsigned current=epoch.load(std::memory_order_acquire);
                if(current!=previous){mixer.clear();previous=current;}
                drain(mixer,current);
                if(!primary.fill(mixer,master.load(std::memory_order_relaxed)))break;
                if(WaitForMultipleObjects(2,waiters,FALSE,100)==WAIT_FAILED)break;
            }
        }
    }
    if(SUCCEEDED(com))CoUninitialize();
    mixer.clear();
    for(;;) {
        DWORD now=GetTickCount();
        unsigned current=epoch.load(std::memory_order_acquire);
        if(current!=previous){cancel();previous=current;}
        if(!device && (!failed || LONG(now-retryAfter)>=0)) {
            failed=!open();backend=failed?"device unavailable":"waveOut";retryAfter=now+1500;
        }
        drain(mixer,current);
        if(!device)mixer.clear();
        for(unsigned i=0;device && i<numBlocks;i++) {
            auto& b=blocks[nextBlock];
            if(b.queued && !(b.header.dwFlags&WHDR_DONE))break;
            if(mixer.active())mixer.render(b.pcm,master.load(std::memory_order_relaxed));else b.pcm.fill(0);
            if(waveOutWrite(device,&b.header,sizeof(b.header))!=MMSYSERR_NOERROR) {
                close();failed=true;retryAfter=now+1500;break;
            }
            b.queued=true; nextBlock=(nextBlock+1)%numBlocks;
        }
        if(WaitForMultipleObjects(2,waiters,FALSE,INFINITE)==WAIT_FAILED)break;
    }
}
DWORD WINAPI run(void*) {
    try { Output output(path);output.loop(); }
    catch(...) { /* Sound files and device availability must not abort the game. */ }
    return 0;
}
}
void report(FILE* file) {
    std::fprintf(file,"Audio=%s WhooshWAVs=%u WindWAVs=%u WindSpeedRatio=%.2f\n",
                 backend.load(),whooshFiles.load(),windFiles.load(),speedRatio.load());
}
void speed(float v) { speedRatio.store(std::isfinite(v) && v>=2?unitRange(v/40.f):0,std::memory_order_relaxed); }
void windVolume(float gain) { windGain.store(gain,std::memory_order_relaxed); }
void start(const std::wstring& root) {
    if(wake)return;
    path=root; wake=CreateEventW(nullptr,FALSE,FALSE,nullptr);
    if(!wake)return;
    HANDLE worker=CreateThread(nullptr,0,run,nullptr,0,nullptr);
    if(worker)CloseHandle(worker);
    else {CloseHandle(wake);wake=nullptr;}
}
void level(float gain) { master.store(unitRange(gain),std::memory_order_relaxed); }
void silence() { speed(0);if(wake){++epoch;SetEvent(wake);} }
void play(Event event) {
    if(!wake || master.load(std::memory_order_relaxed)<=0)return;
    unsigned w=writer.load(std::memory_order_relaxed);
    if(w-reader.load(std::memory_order_acquire)>=queueSize)return;
    queue[w%queueSize]={event,epoch.load(),GetTickCount()};
    writer.store(w+1,std::memory_order_release);SetEvent(wake);
}
}
