# pico-pio-pwm

Note that if a compatibility issue arises when building ```Compatibility with CMake < 3.5 has been removed from CMake.``` when using ```cmake >= 4.0.0``` it might help to change line ```23``` in file ```pico-sdk > lib > mbedtls > CMakeLists.txt``` from ```cmake_minimum_required(VERSION 2.8.12)``` to ```cmake_minimum_required(VERSION 3.5)```
