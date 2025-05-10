cmake_minimum_required(VERSION 3.10)
set(TARGET LTC6810Driver)

project(${TARGET} VERSION 1.0 LANGUAGES CXX)

set(CPP_FILES )

# add_library(${TARGET} STATIC ${CPP_FILES})
add_library(${TARGET} INTERFACE)

target_include_directories(${TARGET} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/Inc)