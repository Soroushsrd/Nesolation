cmake_minimum_required(VERSION 4.0)
project(Nes)

set(CMAKE_CXX_STANDARD 20)

add_executable(Nes main.cpp
        Cpu.cpp
        Cpu.h
        Bus.cpp
        Bus.h)
