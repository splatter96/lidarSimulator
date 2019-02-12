cmake_minimum_required(VERSION 3.5)
project(raytraycer)

find_package(OpenMP)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -O3")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -Wextra")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra")

 if (OPENMP_FOUND)
   set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
   set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
   set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(raytraycer raytracepolymesh.cpp)
