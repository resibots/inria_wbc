cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(tsid_controllers)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TSID_CONTROLLERS_VERSION 1.0.0)

option(COMPILE_ROBOT_DART_EXAMPLE "compile robot-dart example" ON)
option(COMPILE_ROBOT_DART_EXAMPLE_GRAPHICS "compile robot-dart example with graphics" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(tsid REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(xsens_parser)

list(APPEND DEPENDENCIES_LIBS
                             pinocchio::pinocchio
                             tsid::tsid
                             RobotDART::Simu
                             ${YAML_CPP_LIBRARIES}
                            #  ${XSENS_PARSER_LIBRARIES}
                            #  ${XSENS_PARSER_DEPENDENCIES_LIBS}
)

list(APPEND DEPENDENCIES_INCS
                             ${CMAKE_SOURCE_DIR}/include
                             ${XSENS_PARSER_DEPENDENCIES_INCS}
)

set(TSID_CONTROLLERS_HEADERS
                            include/controllers/talos_base_controller.hpp
                            include/controllers/talos_pos_tracking.hpp
                            include/behaviors/factory.hpp
                            include/behaviors/talos_squat.hpp
                            include/behaviors/talos_move_arm.hpp
                            include/utils/trajectory_handler.hpp
                            include/utils/utils.hpp
)

set(TSID_CONTROLLERS_SOURCES
                            src/controllers/talos_base_controller.cpp
                            src/controllers/talos_pos_tracking.cpp
                            src/behaviors/talos_squat.cpp
                            src/behaviors/talos_move_arm.cpp
)

add_library(tsid_controllers SHARED ${TSID_CONTROLLERS_HEADERS} ${TSID_CONTROLLERS_SOURCES})


target_link_libraries(tsid_controllers PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(tsid_controllers
                                           PUBLIC
                                           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/build/include>
                                           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                                           $<INSTALL_INTERFACE:include>
                                           $<BUILD_INTERFACE:${DEPENDENCIES_INCS}>
)


if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    # we ask for Magnum because we want to build the graphics
    if(${COMPILE_ROBOT_DART_EXAMPLE_GRAPHICS} STREQUAL "ON")
        find_package(RobotDART REQUIRED OPTIONAL_COMPONENTS Magnum)
    else()
        find_package(RobotDART REQUIRED)
    endif()

    add_executable(test-controller src/robot_dart/test_controller.cpp)
    target_link_libraries(test-controller PUBLIC tsid_controllers)

    if(RobotDART_Magnum_FOUND)
        add_executable(test-controller-graphics src/robot_dart/test_controller.cpp)
        target_link_libraries(test-controller-graphics PUBLIC tsid_controllers RobotDART::Magnum)
    endif()
endif()



generate_export_header(tsid_controllers)

install(TARGETS tsid_controllers
                               EXPORT targets
                               LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                               INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(FILES ${TSID_CONTROLLERS_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/tsid_controllers/)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(LIBRARY tsid_controllers)

configure_package_config_file("cmake/tsid_controllers.cmake.in"
                              "${CMAKE_CURRENT_BINARY_DIR}/tsid_controllersConfig.cmake"
                              INSTALL_DESTINATION 
                              "${LIBRARY_INSTALL_DIR}/cmake/tsid_controllers"
                              PATH_VARS 
                              INCLUDE_INSTALL_DIR 
                              LIBRARY_INSTALL_DIR LIBRARY
                              DEPENDENCIES_LIBS DEPENDENCIES_INCS
)

install(EXPORT targets
                     FILE
                     tsid_controllersTargets.cmake
                     DESTINATION
                     ${LIBRARY_INSTALL_DIR}/cmake/tsid_controllers
)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/tsid_controllersConfigVersion.cmake"
                                 VERSION ${TSID_CONTROLLERS_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/tsid_controllersConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/tsid_controllersConfigVersion.cmake"
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib/cmake/tsid_controllers
)
