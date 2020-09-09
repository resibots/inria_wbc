find_package(Boost COMPONENTS unit_test_framework)

enable_testing()
set(BINARY ${CMAKE_PROJECT_NAME}_test)

# the tests require Dart to run (for now?)
if(${COMPILE_ROBOT_DART_EXAMPLE} STREQUAL "ON")
        set(TEST_SOURCES test_clone.cpp test_factory.cpp)
endif()

foreach(TEST ${TEST_SOURCES})
        string(REPLACE ".cpp" "" TEST_NAME ${TEST})
        message("TEST: ${TEST_NAME}")
        add_executable(${TEST_NAME} ${TEST})
        add_test(NAME ${TEST_NAME}  COMMAND ${TEST_NAME})
        target_include_directories(${TEST_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
        target_link_libraries(${TEST_NAME} PUBLIC inria_wbc ${Boost_LIBRARIES})
endforeach()


