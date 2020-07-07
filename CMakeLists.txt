cmake_minimum_required(VERSION 3.0)
project(opensot_visual_servoing)

add_compile_options(-std=c++11)

option(COMPILE_TESTS "Compile tests" FALSE)

find_package(cartesian_interface)
find_package(catkin REQUIRED COMPONENTS cartesian_interface roscpp urdf)
find_package(OpenSoT REQUIRED)
find_package(VISP REQUIRED)

###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

    INCLUDE_DIRS

    LIBRARIES
    OpenSoTVisualServoingTask

    )

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
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h")

if(COMPILE_TESTS)
  enable_testing()
    add_subdirectory(tests)
endif()



