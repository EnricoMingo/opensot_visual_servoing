cmake_minimum_required(VERSION 3.0)
project(opensot_visual_servoing)

add_compile_options(-std=c++11)

option(COMPILE_TESTS "Compile tests" FALSE)

find_package(catkin REQUIRED COMPONENTS cartesian_interface roscpp urdf)
find_package(OpenSoT REQUIRED)
find_package(VISP REQUIRED)

catkin_package()

include_directories(include
    ${OpenSoT_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS})

# cartesio_acceleration_support library
add_library(OpenSoTVisualServoingTask SHARED
    src/tasks/velocity/VisualServoing.cpp)

target_link_libraries(OpenSoTVisualServoingTask
    ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${VISP_LIBRARIES})

install(TARGETS OpenSoTVisualServoingTask
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

if(COMPILE_TESTS)
  enable_testing()
    add_subdirectory(tests)
endif()



