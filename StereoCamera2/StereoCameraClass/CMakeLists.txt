cmake_minimum_required(VERSION 3.6)
project(StereoCameraClass)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        extrinsics.yml
        intrinsics.yml
        main.cpp
        stdafx.h
        StereoCamera.cpp
        StereoCamera.h)

add_executable(StereoCameraClass ${SOURCE_FILES})
