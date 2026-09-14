"""Per-language standard flags for the ESP32 env.

platformio.ini's build_flags is appended to BOTH CFLAGS and CXXFLAGS, so it
cannot carry -std=gnu11 (would break every Arduino .cpp) or -std=gnu++11
(meaningless to gcc for .c). This script is the only correct place to give
C and C++ different standards in one PlatformIO environment.
"""
Import("env")

env.Append(CFLAGS=["-std=gnu11", "-Wall", "-Wextra",
                   "-Wno-unused-parameter", "-Wno-expansion-to-defined"])
env.Append(CXXFLAGS=["-std=gnu++11"])
