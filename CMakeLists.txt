cmake_minimum_required(VERSION 3.20)
project(sac LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

# --- library: sac_attitude -------------------------------------------------

add_library(sac_attitude
    src/attitude/Rotation.cpp
    src/attitude/Quaternion.cpp
)
target_include_directories(sac_attitude PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(sac_attitude PUBLIC
    Eigen3::Eigen
)

# --- tests: test_rotation ---------------------------------------------------

add_executable(test_rotation
    tests/attitude/test_rotation.cpp
)
target_link_libraries(test_rotation PRIVATE
    sac_attitude
    GTest::gtest
    GTest::gtest_main
)

enable_testing()
add_test(NAME test_rotation COMMAND test_rotation)


# --- tests: test_quaternion -------------------------------------------------

add_executable(test_quaternion
    tests/attitude/test_quaternion.cpp
)
target_link_libraries(test_quaternion PRIVATE
    sac_attitude
    GTest::gtest
    GTest::gtest_main
)

add_test(NAME test_quaternion COMMAND test_quaternion)