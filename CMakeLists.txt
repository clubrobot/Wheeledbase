cmake_minimum_required(VERSION 4.2)
project(wheeledbase)

add_library(wheeledbase STATIC src/Wheeledbase.cpp)
target_include_directories(wheeledbase
        PUBLIC
            include
            include/Controller
            include/MoveStrategy
        PRIVATE
            lib
            lib/Codewheel
            lib/PeriodicProcess
            lib/PID
            lib/Odometry
            lib/mathutils
            lib/Logger
            lib/motors/include
)
target_compile_features(wheeledbase PUBLIC cxx_std_23)