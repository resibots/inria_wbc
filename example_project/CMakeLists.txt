cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(inria_wbc_com)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INRIA_WBC_TELEOP_VERSION 1.0.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -std=c++14")

find_package(inria_wbc REQUIRED)

find_path(LIMBO_INCLUDE_DIR limbo/limbo.hpp ENV LIMBO_PATH)

if(NOT ${LIMBO_INCLUDE_DIR} MATCHES "LIMBO_INCLUDE_DIR-NOTFOUND")
    message("Found limbo: ${LIMBO_INCLUDE_DIR}")
else()
    message("-- limbo: not found (use -DLIMBO_INCLUDE_DIR)")
endif()

list(APPEND DEPENDENCIES_LIBS
                             ${INRIA_WBC_LIBRARIES}
                             ${INRIA_WBC_LIBRARIES}
                             ${INRIA_WBC_DEPENDENCIES_LIBS}
)

list(APPEND DEPENDENCIES_INCS
                             ${CMAKE_SOURCE_DIR}/include
                             ${INRIA_WBC_INCLUDE_DIRS}
                             ${LIMBO_INCLUDE_DIR}
                             ${INRIA_WBC_DEPENDENCIES_INCS}
                             ${COMPLIANT_STABILIZER_INCLUDE_DIRS}
)

set(INRIA_WBC_COM_HEADERS
                            include/inria_wbc/behaviors/test_wbc_com.hpp
                            include/inria_wbc/behaviors/talos_walk_on_spot_stabilized.hpp
                            include/inria_wbc/behaviors/talos_large_move_arm.hpp
                            include/inria_wbc/behaviors/talos_bending.hpp
)

set(INRIA_WBC_COM_SOURCES
                            src/behaviors/test_wbc_com.cpp
                            src/behaviors/talos_walk_on_spot_stabilized.cpp
                            src/behaviors/talos_large_move_arm.cpp
                            src/behaviors/talos_bending.cpp
)

set(EXAMPLE_LIST 
                src/robot_dart/test_controller.cpp
)

if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    list(APPEND DEPENDENCIES_LIBS RobotDART::Simu)
endif()

add_library(inria_wbc_com SHARED ${INRIA_WBC_COM_HEADERS} ${INRIA_WBC_COM_SOURCES})

target_link_libraries(inria_wbc_com PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(inria_wbc_com
                                           PUBLIC
                                           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/build/include>
                                           $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                                           $<INSTALL_INTERFACE:include>
                                           $<BUILD_INTERFACE:${DEPENDENCIES_INCS}>
)

function(compile_examples)
    foreach(EXAMPLE ${ARGN})
        string(REPLACE ".cpp" "" EXAMPLE_NAME ${EXAMPLE} )
        string(REPLACE "src/robot_dart/" "" EXAMPLE_NAME ${EXAMPLE_NAME} )
        message("COMPILING EXAMPLE ${EXAMPLE_NAME}")
        add_executable(${EXAMPLE_NAME} ${EXAMPLE})
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc_com)
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc Boost::program_options)
        if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            SET_TARGET_PROPERTIES(${EXAMPLE_NAME}  PROPERTIES LINK_FLAGS -Wl,-no-as-needed)
        endif()

        if(RobotDART_Magnum_FOUND)
            add_executable(${EXAMPLE_NAME}_graphics ${EXAMPLE})
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc RobotDART::Magnum Boost::program_options)
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc_com RobotDART::Magnum)
        endif()
    endforeach()
endfunction()

if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    find_package(Boost REQUIRED COMPONENTS program_options)
    compile_examples(${EXAMPLE_LIST})
endif()

generate_export_header(inria_wbc_com)

install(TARGETS inria_wbc_com
                               EXPORT targets
                               LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                               INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(DIRECTORY include/inria_wbc_com DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(LIBRARY inria_wbc_com)

configure_package_config_file("cmake/inria_wbc_com.cmake.in"
                              "${CMAKE_CURRENT_BINARY_DIR}/inria_wbc_com.cmake"
                              INSTALL_DESTINATION 
                              "${LIBRARY_INSTALL_DIR}/cmake/inria_wbc_com"
                              PATH_VARS 
                              INCLUDE_INSTALL_DIR 
                              LIBRARY_INSTALL_DIR LIBRARY
                              DEPENDENCIES_LIBS DEPENDENCIES_INCS
)

install(EXPORT targets
                     FILE
                     inria_wbc_teleopTargets.cmake
                     DESTINATION
                     ${LIBRARY_INSTALL_DIR}/cmake/inria_wbc_teleop
)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/inria_wbc_comConfigVersion.cmake"
                                 VERSION ${INRIA_WBC_TELEOP_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/inria_wbc_comConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/inria_wbc_comConfigVersion.cmake"
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib/cmake/inria_wbc_com
)
