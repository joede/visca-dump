cmake_minimum_required(VERSION 3.1.0)

project(visca-dump)

## Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Populate a CMake variable with the sources
set(viscadump_SRCS
    visca-dump.c
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EZV24 REQUIRED libezV24)


## Tell CMake to create the visca-dump executable
add_executable(visca-dump ${viscadump_SRCS})

## Which libraries do we need...
target_link_libraries(visca-dump ${EZV24_LIBRARIES})
