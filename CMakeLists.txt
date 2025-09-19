cmake_minimum_required(VERSION 4.0)
project(Nes)

set(CMAKE_CXX_STANDARD 20)

add_executable(Nes
        Cpu.cpp
        Cpu.h
        Bus.cpp
        Bus.h
        Cartridge.h
        Cartridge.cpp
        Ppu.h
        Ppu.cpp
        olcNes_Video2_2C02.cpp
        olcPixelGameEngine.h
        Mapper.h
        Mapper.cpp
        Mapper_00.h
        Mapper_00.cpp
)


target_link_libraries(Nes
        X11
        GL
        png
        pthread)