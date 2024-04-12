cmake_minimum_required(VERSION 3.5)

# 工程名称、语言、版本
project(DucoCobotAPI_demo
  LANGUAGES C CXX
  VERSION 0.1.0
  )
include_directories("/usr/local/lib")

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set( demo_src
   DucoCobotAPI_demo/DucoCobot.h
   DucoCobotAPI_demo/DucoCobotAPI_demo.cpp
)



add_executable(DucoCobotAPI_demo ${demo_src} )
target_link_libraries(DucoCobotAPI_demo DucoCobotAPI pthread)



