cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(inria_wbc)

include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(INRIA_WBC_VERSION 1.0.0)

option(COMPILE_ROBOT_DART_EXAMPLE "compile robot-dart example" ON)
option(COMPILE_ROBOT_DART_EXAMPLE_GRAPHICS "compile robot-dart example with graphics" ON)

find_package(tsid REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Utheque) # optional
find_package(Boost OPTIONAL_COMPONENTS stacktrace_basic)


set(INRIA_WBC_SOURCES
    src/controllers/controller.cpp
    src/controllers/pos_tracker.cpp
    src/controllers/tasks.cpp
    src/controllers/humanoid_pos_tracker.cpp
    src/controllers/talos_pos_tracker.cpp
    src/trajs/saver.cpp
    src/trajs/loader.cpp
    src/behaviors/behavior.cpp
    src/behaviors/generic/cartesian.cpp
    src/behaviors/generic/cartesian_traj.cpp
    src/behaviors/humanoid/move_com.cpp
    src/behaviors/humanoid/move_feet.cpp
    src/behaviors/humanoid/walk_on_spot.cpp
    src/behaviors/humanoid/walk.cpp
    src/behaviors/humanoid/active_walk.cpp
    src/behaviors/humanoid/clapping.cpp
    src/safety/torque_collision_detection.cpp
    src/stabilizers/stabilizer.cpp
    src/estimators/cop.cpp
    src/safety/collision_check.cpp
    src/tsid/task-self-collision.cpp
    src/tsid/task-momentum-equality.cpp
    src/utils/robot_model.cpp
)

add_library(inria_wbc SHARED ${INRIA_WBC_SOURCES})
target_compile_definitions(inria_wbc PUBLIC BOOST_MATH_DISABLE_FLOAT128=1)
IF(APPLE)
    target_compile_definitions(inria_wbc PUBLIC _GNU_SOURCE=1)
endif()
target_compile_features(inria_wbc PUBLIC cxx_std_14)

if (Utheque_FOUND)
  target_compile_definitions(inria_wbc PUBLIC WBC_HAS_UTHEQUE)
  set(UTHEQUE_LIB Utheque)
endif()

target_link_libraries(inria_wbc PUBLIC 
			pinocchio::pinocchio
			tsid::tsid
			${YAML_CPP_LIBRARIES}
      ${UTHEQUE_LIB})

if(Boost_stacktrace_basic_FOUND)
  target_compile_definitions(inria_wbc PUBLIC IWBC_USE_STACKTRACE=1 BOOST_STACKTRACE_DYN_LINK=1) #force to not use header library but shared
  target_link_libraries(inria_wbc PUBLIC Boost::stacktrace_basic)
else()
  message(STATUS "Could NOT find Boost stacktrace")
endif()

target_include_directories(inria_wbc PUBLIC
		$<BUILD_INTERFACE:${inria_wbc_BINARY_DIR}/include>
		$<BUILD_INTERFACE:${inria_wbc_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

generate_export_header(inria_wbc)

install(TARGETS inria_wbc EXPORT targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

configure_package_config_file("cmake/inria_wbc.cmake.in" "inria_wbcConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/inria_wbc"
  PATH_VARS
)

install(EXPORT targets
  FILE inria_wbcTargets.cmake
  NAMESPACE inria_wbc::
  DESTINATION lib/cmake/inria_wbc
)

write_basic_package_version_file("inria_wbcConfigVersion.cmake"
  VERSION ${INRIA_WBC_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES 
	${inria_wbc_BINARY_DIR}/inria_wbcConfig.cmake 
	${inria_wbc_BINARY_DIR}/inria_wbcConfigVersion.cmake
        DESTINATION lib/cmake/inria_wbc
)

install(DIRECTORY etc/  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/inria_wbc/etc)


# generate robot dart tests
add_subdirectory(src/robot_dart)


enable_testing()
# this is you happen to use "make test" (don't)
list(APPEND CMAKE_CTEST_ARGUMENTS "--output-on-failure")
# the right way of running the test is "make check"
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
     --force-new-ctest-process
     --verbose
      --output-on-failure
)

add_subdirectory(tests)
