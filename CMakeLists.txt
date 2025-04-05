cmake_minimum_required(VERSION 3.10)
set(TARGET LTC6810Driver)

project(${TARGET} VERSION 1.0 LANGUAGES CXX)

set(CPP_FILES
    Src/StateMachine.cpp
    Src/Driver.cpp
)

add_library(${TARGET} STATIC ${CPP_FILES})

target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Inc)

target_compile_features(${TARGET} PRIVATE cxx_std_23)
