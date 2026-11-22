#pragma once
#include <cstdio>
namespace whoosh::visuals {
struct Options {
    int blur=2;
    bool wind=true, lights=true, flares=true;
    float strength=.55f, windStart=120.f/3.6f, windOpacity=1;
};
void configure(Options value);
void setTexturePath(const wchar_t* path);
void install();
void report(FILE* file);
void driving(bool active);
}
