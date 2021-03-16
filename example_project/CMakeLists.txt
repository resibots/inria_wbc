cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(@project_name@)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(@PROJECT_NAME@_VERSION 1.0.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -std=c++14")

option(COMPILE_ROBOT_DART_EXAMPLES "Compile Robot DART examples" ON)

find_package(inria_wbc REQUIRED)

list(APPEND DEPENDENCIES_LIBS
    inria_wbc::inria_wbc
)

set(@PROJECT_NAME@_SOURCES
    src/behaviors/ex_behavior.cpp
    src/controllers/ex_controller.cpp
    src/tsid/ex_task.cpp

)

set(EXAMPLE_LIST 
    src/robot_dart/test_controller.cpp
)

add_library(@project_name@ SHARED ${@PROJECT_NAME@_SOURCES})

target_compile_definitions(@project_name@ PUBLIC BOOST_MATH_DISABLE_FLOAT128=1)
target_compile_features(@project_name@ PUBLIC cxx_std_14)

target_link_libraries(@project_name@ PUBLIC ${DEPENDENCIES_LIBS})

target_include_directories(@project_name@
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/build/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



######################### INSTALLATION #########################################

generate_export_header(@project_name@)

install(TARGETS @project_name@
        EXPORT targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

configure_package_config_file("cmake/@project_name@.cmake.in" "@project_name@Config.cmake"
                              INSTALL_DESTINATION "lib/cmake/@project_name@"
                              PATH_VARS
)

# CHANGE NAMESPACE AS YOU PREFER. The project is now linked as inria_wbc::@project_name@
install(EXPORT targets
        FILE @project_name@Targets.cmake
        NAMESPACE inria_wbc::
        DESTINATION lib/cmake/@project_name@
)

write_basic_package_version_file("@project_name@ConfigVersion.cmake"
                                 VERSION ${@PROJECT_NAME@_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/@project_name@Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/@project_name@ConfigVersion.cmake"
        DESTINATION lib/cmake/@project_name@
)


########################## TEST EXAPLES USING ROBOT DART #########################

function(compile_examples)
    foreach(EXAMPLE ${ARGN})
        string(REPLACE ".cpp" "" EXAMPLE_NAME ${EXAMPLE} )
        string(REPLACE "src/robot_dart/" "" EXAMPLE_NAME ${EXAMPLE_NAME} )
        message("COMPILING EXAMPLE ${EXAMPLE_NAME}")
        add_executable(${EXAMPLE_NAME} ${EXAMPLE})
        target_link_libraries(${EXAMPLE_NAME} PUBLIC @project_name@)
        target_link_libraries(${EXAMPLE_NAME} PUBLIC inria_wbc::inria_wbc RobotDART::Simu Boost::program_options)
        if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            target_link_options(${EXAMPLE_NAME} PRIVATE "LINKER:--no-as-needed")
        endif()

        if(RobotDART_Magnum_FOUND)
            add_executable(${EXAMPLE_NAME}_graphics ${EXAMPLE})
            if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                target_link_options(${EXAMPLE_NAME}_graphics PRIVATE "LINKER:--no-as-needed")
            endif()
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC inria_wbc::inria_wbc RobotDART::Simu RobotDART::Magnum Boost::program_options)
            target_link_libraries(${EXAMPLE_NAME}_graphics PUBLIC @project_name@)
        endif()
    endforeach()
endfunction()

if(${COMPILE_ROBOT_DART_EXAMPLES} STREQUAL "ON")
    find_package(RobotDART COMPONENTS Simu OPTIONAL_COMPONENTS Magnum)
    find_package(Boost REQUIRED COMPONENTS program_options)
    compile_examples(${EXAMPLE_LIST})
endif()


