#pragma once
#include <d3d9.h>
#include "motion.hpp"

namespace whoosh::visuals::carbon {
template<class T> struct LocalRef {
    T* p{};
    ~LocalRef(){if(p)p->Release();}
    LocalRef()=default;
    LocalRef(const LocalRef&)=delete;
};

struct BlurVertex {float x,y,z,rhw; DWORD diffuse; UV uv;};
static_assert(sizeof(BlurVertex)==28);

// RenderWare camera textures participate in the game's D3D9 lost/reset lifecycle.
// No surface, state block or bound texture references are retained across frames.
struct Targets {
    uintptr_t raster[2]{};
    int width{},height{};
    bool resize(int w,int h) {
        if(w==width && h==height && raster[0] && raster[1])return true;
        for(auto& target:raster){
            if(target)reinterpret_cast<int(__cdecl*)(uintptr_t)>(0x7fb020)(target);
            target=0;
        }
        width=w;height=h;
        for(auto& target:raster) {
            target=reinterpret_cast<uintptr_t(__cdecl*)(int,int,int,int)>(0x7fb230)(w,h,32,0x0505);
            if(!target)return false;
        }
        return true;
    }
};
inline uintptr_t filter(uintptr_t capture,int w,int h,int bw,int bh,
                         Vec motion,float exposure,float radial,bool useCarbon,const char*& stage,HRESULT& result) {
    stage="raster extension";
    // RenderWare caches stage bindings. Read the real texture from its D3D9 extension.
    int ext=*reinterpret_cast<const int*>(0xB4E9E0);
    if(ext<52 || ext>4096)return 0;
    auto acquire=[ext](uintptr_t raster,LocalRef<IDirect3DBaseTexture9>& ref) {
        std::memcpy(&ref.p,reinterpret_cast<const void*>(raster+ext),sizeof(ref.p));
        if(!ref.p)return false;
        ref.p->AddRef();
        return ref.p->GetType()==D3DRTYPE_TEXTURE;
    };
    stage="D3D9 device";
    auto device=*reinterpret_cast<IDirect3DDevice9**>(0xC97C28);
    if(!device)return 0;
    stage="allocate camera textures";
    static Targets targets;
    if(!targets.resize(std::max(1,w/2),std::max(1,h/2)))return 0;
    stage="get render surfaces";
    LocalRef<IDirect3DBaseTexture9> input,texture[2];
    LocalRef<IDirect3DSurface9> surface[2],back,depth;
    LocalRef<IDirect3DStateBlock9> state;
    if(FAILED(result=device->GetRenderTarget(0,&back.p)) ||
       !acquire(capture,input))return 0;
    HRESULT depthResult=device->GetDepthStencilSurface(&depth.p);
    if(FAILED(depthResult) && depthResult!=D3DERR_NOTFOUND){result=depthResult;return 0;}
    for(unsigned i=0;i<2;++i) {
        bool ok=acquire(targets.raster[i],texture[i]) &&
                SUCCEEDED(result=static_cast<IDirect3DTexture9*>(texture[i].p)->GetSurfaceLevel(0,&surface[i].p));

        if(!ok)return 0;
    }
    stage="save render state";
    if(FAILED(result=device->CreateStateBlock(D3DSBT_ALL,&state.p)))return 0;
    D3DVIEWPORT9 originalViewport{};
    if(FAILED(result=device->GetViewport(&originalViewport)))return 0;
    D3DCAPS9 caps{};
    if(FAILED(result=device->GetDeviceCaps(&caps)))return 0;
    LocalRef<IDirect3DSurface9> auxiliary[3];
    const DWORD slots=std::min<DWORD>(caps.NumSimultaneousRTs,4);
    for(DWORD slot=1;slot<slots;++slot) {
        HRESULT hr=device->GetRenderTarget(slot,&auxiliary[slot-1].p);
        if(FAILED(hr) && hr!=D3DERR_NOTFOUND){result=hr;return 0;}
    }
    if(FAILED(result=device->SetDepthStencilSurface(nullptr)))return 0;
    bool ok=true;
    for(DWORD slot=1;slot<slots && ok;++slot)
        if(auxiliary[slot-1].p)ok=SUCCEEDED(result=device->SetRenderTarget(slot,nullptr));
    stage="bind spatial filter";
    ok=ok && SUCCEEDED(result=device->SetVertexShader(nullptr)) &&
            SUCCEEDED(result=device->SetPixelShader(nullptr)) &&
            SUCCEEDED(result=device->SetFVF(D3DFVF_XYZRHW|D3DFVF_DIFFUSE|D3DFVF_TEX1));
    auto stageState=[&](DWORD index,D3DTEXTURESTAGESTATETYPE type,DWORD value) {
        if(ok)ok=SUCCEEDED(result=device->SetTextureStageState(index,type,value));
    };
    stageState(0,D3DTSS_TEXCOORDINDEX,0);
    stageState(0,D3DTSS_TEXTURETRANSFORMFLAGS,D3DTTFF_DISABLE);
    stageState(0,D3DTSS_COLOROP,D3DTOP_SELECTARG1);
    stageState(0,D3DTSS_COLORARG1,D3DTA_TEXTURE);
    stageState(0,D3DTSS_ALPHAOP,D3DTOP_SELECTARG1);
    stageState(0,D3DTSS_ALPHAARG1,D3DTA_DIFFUSE);
    stageState(0,D3DTSS_RESULTARG,D3DTA_CURRENT);
    stageState(1,D3DTSS_COLOROP,D3DTOP_DISABLE);
    stageState(1,D3DTSS_ALPHAOP,D3DTOP_DISABLE);
    device->SetRenderState(D3DRS_ZENABLE,FALSE);
    device->SetRenderState(D3DRS_ZWRITEENABLE,FALSE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE,TRUE);
    device->SetRenderState(D3DRS_BLENDOP,D3DBLENDOP_ADD);
    device->SetRenderState(D3DRS_SRCBLEND,D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND,D3DBLEND_INVSRCALPHA);
    device->SetRenderState(D3DRS_FOGENABLE,FALSE);
    device->SetRenderState(D3DRS_LIGHTING,FALSE);
    device->SetRenderState(D3DRS_FILLMODE,D3DFILL_SOLID);
    device->SetRenderState(D3DRS_SRGBWRITEENABLE,FALSE);
    device->SetSamplerState(0,D3DSAMP_SRGBTEXTURE,FALSE);
    device->SetRenderState(D3DRS_ALPHATESTENABLE,FALSE);
    device->SetRenderState(D3DRS_SCISSORTESTENABLE,FALSE);
    device->SetRenderState(D3DRS_STENCILENABLE,FALSE);
    device->SetRenderState(D3DRS_CULLMODE,D3DCULL_NONE);
    device->SetRenderState(D3DRS_COLORWRITEENABLE,7);
    device->SetSamplerState(0,D3DSAMP_ADDRESSU,D3DTADDRESS_CLAMP);
    device->SetSamplerState(0,D3DSAMP_ADDRESSV,D3DTADDRESS_CLAMP);
    device->SetSamplerState(0,D3DSAMP_MINFILTER,D3DTEXF_LINEAR);
    device->SetSamplerState(0,D3DSAMP_MAGFILTER,D3DTEXF_LINEAR);
    device->SetSamplerState(0,D3DSAMP_MIPFILTER,D3DTEXF_NONE);
    // Blend the successive samples of this pass as a running weighted mean.
    // Alpha belongs to the coverage mesh in the later composite, so keep it opaque here.
    const unsigned passes=useCarbon ? (radial>0.f ? 5 : 4) : 3;
    for(unsigned pass=0;pass<passes && ok;++pass) {
        stage="bind spatial pass target";
        const unsigned dest=pass%2;
        D3DSURFACE_DESC desc{};
        ok=SUCCEEDED(result=surface[dest].p->GetDesc(&desc)) &&
           SUCCEEDED(result=device->SetTexture(0,nullptr)) &&
           SUCCEEDED(result=device->SetRenderTarget(0,surface[dest].p)) &&
           SUCCEEDED(result=device->SetTexture(0,pass?texture[1-dest].p:input.p));
        if(!ok)break;
        stage="clear spatial target";
        ok=SUCCEEDED(result=device->Clear(0,nullptr,D3DCLEAR_TARGET,0xff000000,1,0));
        if(!ok)break;
        const unsigned taps=pass ? 8 : 1;
        BlurVertex vertices[8*6];
        for(unsigned tap=0;tap<taps;++tap) {
            BlurVertex quad[4];
            for(unsigned i=0;i<4;++i) {
                UV uv{float(i&1),float(i>>1)};
                UV sample=pass ? spatialTap(uv,motion,exposure,useCarbon,pass,tap,desc.Width,desc.Height)
                               : UV{uv.x*w/bw,uv.y*h/bh};
                if(pass==4) {
                    // Sample towards the optical centre. The middle pixel stays fixed
                    // even when the player is looking sideways or backwards.
                    sample=nitrousRadialTap(uv,radial,tap);
                }
                quad[i]={uv.x*desc.Width-.5f,uv.y*desc.Height-.5f,0,1,
                         (spatialAlpha(useCarbon,tap)<<24)|0xffffffu,sample};
            }
            unsigned n=tap*6;
            for(unsigned corner:{0u,1u,2u,2u,1u,3u})vertices[n++]=quad[corner];
        }
        stage="draw spatial pass";
        ok=SUCCEEDED(result=device->DrawPrimitiveUP(D3DPT_TRIANGLELIST,taps*2,vertices,sizeof(BlurVertex)));
    }
    device->SetTexture(0,nullptr);
    device->SetRenderTarget(0,back.p);
    for(DWORD slot=1;slot<slots;++slot)
        if(auxiliary[slot-1].p)device->SetRenderTarget(slot,auxiliary[slot-1].p);
    device->SetDepthStencilSurface(depth.p);
    state.p->Apply();
    device->SetViewport(&originalViewport);
    return ok?targets.raster[(passes-1)%2]:0;
}
}
