cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(tsid_controllers)

option(BUILD_GRAPHICS "Build  graphics" ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(tsid REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

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
                    ${YAML_CPP_LIBRARIES}
)

list(APPEND DEPENDENCIES_INCS
                    ${CMAKE_SOURCE_DIR}/include
)


set(TSID_CONTROLLERS_HEADERS
                    include/controllers/talos_base_controller.hpp
                    include/controllers/talos_pos_tracking.hpp
                    include/examples/factory.hpp
                    include/examples/talos_squat.hpp
                    include/utils/trajectory_handler.hpp
                    include/utils/utils.hpp
)

set(TSID_CONTROLLERS_SOURCES
                    src/controllers/talos_base_controller.cpp
                    src/controllers/talos_pos_tracking.cpp
                    src/examples/talos_squat.cpp

)

add_library(tsid_controllers SHARED
                    ${TSID_CONTROLLERS_HEADERS}
                    ${TSID_CONTROLLERS_SOURCES}
)


target_link_libraries(tsid_controllers PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(tsid_controllers
                                PUBLIC
                                $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/build/include>
                                $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                                $<INSTALL_INTERFACE:include>
                                ${DEPENDENCIES_INCS}
)



add_executable(test-controller src/robot_dart/test_controller.cpp)
target_link_libraries(test-controller PUBLIC tsid_controllers)

if(RobotDART_Magnum_FOUND)
    add_executable(test-controller-graphics src/robot_dart/test_controller.cpp)
    target_link_libraries(test-controller-graphics PUBLIC tsid_controllers RobotDART::Magnum)
endif()

