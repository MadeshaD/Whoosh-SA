#pragma once
#include "detector.hpp"
#include <string>
#include <cstdio>
namespace whoosh::audio {
void report(FILE* file);
void start(const std::wstring& root);
void windVolume(float gain);
void speed(float metresPerSecond);
void level(float gain);
void silence();
void play(Event event);
}
