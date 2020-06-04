cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(tsid_talos_dart)

option(BUILD_GRAPHICS "Build  graphics" ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(tsid REQUIRED)
find_package(pinocchio REQUIRED)
# we ask for Magnum because we want to build the graphics

if(BUILD_GRAPHICS)
    find_package(RobotDART REQUIRED OPTIONAL_COMPONENTS Magnum)
else()
    find_package(RobotDART REQUIRED)
endif()


list(APPEND DEPENDENCIES_LIBS
                    pinocchio::pinocchio
                    tsid::tsid
                    RobotDART::Simu
)

list(APPEND DEPENDENCIES_INCS
                    ${CMAKE_SOURCE_DIR}/include
)

add_executable(talos src/talos.cpp)
target_link_libraries(talos PUBLIC ${DEPENDENCIES_LIBS})
target_include_directories(talos  PUBLIC ${DEPENDENCIES_INCS})

if(RobotDART_Magnum_FOUND)
    list(APPEND DEPENDENCIES_LIBS RobotDART::Magnum)
    add_executable(talos_graphics src/talos.cpp)
    target_link_libraries(talos_graphics PUBLIC ${DEPENDENCIES_LIBS})
    target_include_directories(talos_graphics  PUBLIC ${DEPENDENCIES_INCS})
endif()
