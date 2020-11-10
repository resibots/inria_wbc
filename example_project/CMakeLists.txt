cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(@project_name@)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(@PROJECT_NAME@_VERSION 1.0.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -std=c++14")

find_package(inria_wbc REQUIRED)

list(APPEND DEPENDENCIES_LIBS
                             ${INRIA_WBC_LIBRARIES}
                             ${INRIA_WBC_LIBRARIES}
                             ${INRIA_WBC_DEPENDENCIES_LIBS}
)

list(APPEND DEPENDENCIES_INCS
                             ${CMAKE_SOURCE_DIR}/include
                             ${INRIA_WBC_INCLUDE_DIRS}
                             ${INRIA_WBC_DEPENDENCIES_INCS}
)

set(@PROJECT_NAME@_HEADERS
                            include/inria_wbc/behaviors/ex_behavior.hpp
                            include/inria_wbc/controllers/ex_controller.hpp
                            include/tsid/tasks/ex_task.hpp

)

set(@PROJECT_NAME@_SOURCES
                            src/behaviors/ex_behavior.cpp
                            src/controllers/ex_controller.cpp
                            src/tsid/ex_task.cpp

)

set(EXAMPLE_LIST 
                src/robot_dart/test_controller.cpp
)

if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    list(APPEND DEPENDENCIES_LIBS RobotDART::Simu)
endif()

add_library(@project_name@ SHARED ${@PROJECT_NAME@_HEADERS} ${@PROJECT_NAME@_SOURCES})

target_link_libraries(@project_name@ PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(@project_name@
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
        target_link_libraries(${EXAMPLE_NAME} PUBLIC @project_name@)
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc Boost::program_options)
        if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            SET_TARGET_PROPERTIES(${EXAMPLE_NAME}  PROPERTIES LINK_FLAGS -Wl,-no-as-needed)
        endif()

        if(RobotDART_Magnum_FOUND)
            add_executable(${EXAMPLE_NAME}_graphics ${EXAMPLE})
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc RobotDART::Magnum Boost::program_options)
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC @project_name@ RobotDART::Magnum)
        endif()
    endforeach()
endfunction()

if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
    find_package(Boost REQUIRED COMPONENTS program_options)
    compile_examples(${EXAMPLE_LIST})
endif()

generate_export_header(@project_name@)

install(TARGETS @project_name@
                               EXPORT targets
                               LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
                               RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
                               INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

install(DIRECTORY include/@project_name@ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib)
set(LIBRARY @project_name@)

configure_package_config_file("cmake/@project_name@.cmake.in"
                              "${CMAKE_CURRENT_BINARY_DIR}/@project_name@.cmake"
                              INSTALL_DESTINATION 
                              "${LIBRARY_INSTALL_DIR}/cmake/@project_name@"
                              PATH_VARS 
                              INCLUDE_INSTALL_DIR 
                              LIBRARY_INSTALL_DIR LIBRARY
                              DEPENDENCIES_LIBS DEPENDENCIES_INCS
)

install(EXPORT targets
                     FILE
                     @project_name@Targets.cmake
                     DESTINATION
                     ${LIBRARY_INSTALL_DIR}/cmake/@project_name@
)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/@project_name@ConfigVersion.cmake"
                                 VERSION ${@PROJECT_NAME@_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/@project_name@Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/@project_name@ConfigVersion.cmake"
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib/cmake/@project_name@
)
