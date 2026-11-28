<img width="1920" alt="Whoosh Logo" src="https://github.com/user-attachments/assets/ed915972-c9bf-458d-81d8-f5592eda5f87" />

# Whoosh — Need for Speed-style effects
**Whoosh** adds several sound and visual effects inspired by NFS Most Wanted and Carbon to GTA San Andreas.

## Main features:
- **Pass-by sounds.** Passing poles, trees, road signs, oncoming traffic, bridges, and tunnels triggers a “Whoosh” sound. Its volume depends on vehicle speed and distance to the object.
- **Wind noise.** A wind sound plays at high speeds and grows louder as the vehicle accelerates.
- **NFS-style camera profiles.** Choose between the Most Wanted and Carbon profiles. The camera responds to acceleration, braking, and impacts.
- **Motion blur.** Two Motion Blur profiles, inspired by Most Wanted and Carbon, blur the surroundings according to vehicle speed and camera direction.
- **Visual trails.** Thin wind streaks appear behind the vehicle at high speeds. When nitro is active, short light trails follow the illuminated rear lights. Streetlights leave streaks as you pass them.

## Installation and controls
Move the folder containing the files from the archive into your `modloader` folder or the game directory. By default, **F9** reloads the INI file while the game is running.

## Note
This plugin was generated entirely with AI.

## Build and verification

Use Visual Studio C++ with CMake. From the source directory:

    cmake -S . -B build -A Win32
    cmake --build build --config Release

A static 32-bit MinGW-w64 build can also be made directly:

    i686-w64-mingw32-g++ -std=c++17 -O2 -mstackrealign -shared -static -s -Wl,--exclude-all-symbols source/*.cpp -lwinmm -luser32 -lole32 -o Whoosh.SA.asi

The portable tests use the production detector and PCM mixer. With a native C++17 compiler, Python 3, the LAe2 Z-up collision OBJ and the sound folder from the runtime archive:

    c++ -std=c++17 -O2 tests/scene_runner.cpp source/detector.cpp -o scene_runner
    python3 tests/routes.py ./scene_runner path/to/LAe2_collision.obj
    c++ -std=c++17 -O2 tests/traffic_check.cpp source/detector.cpp -o traffic_check
    ./traffic_check
    c++ -std=c++17 -O2 tests/mixer_check.cpp source/mixer.cpp -o mixer_check
    ./mixer_check "path/to/Whoosh SFX/Whoosh"

The camera tests cover acceleration, braking, impacts, directional tracking and mouse handoff at 25, 30, 60 and 144 FPS. The arena check casts 1,536 chase-camera rays at a street, a bridge, a tunnel and a parking garage. To reproduce them:

    c++ -std=c++17 -O2 tests/camera_check.cpp source/camera.cpp -o camera_check
    ./camera_check
    python3 tests/camera_arena.py ./scene_runner path/to/LAe2_collision.obj
    python3 tests/camera_data.py path/to/MW/GLOBAL/attributes.bin path/to/Carbon/GLOBAL/attributes.bin

The 21 LAe2 passes cover both directions along a lamp and fence, the approach to a tunnel, the reverse route, the reference bridge, a covered stretch of the car park, speed and distance cutoffs, and 25/60/144 FPS sampling. The offline runner reads the world-space collision mesh and estimates compact-prop bounds from its vertices. The ASI reads the engine's actual collision boxes and instance transforms. LAe2 is only a test input and is not used by the installed plug-in.

StreakFlares collects the active model-light and traffic-signal registrations each frame.
