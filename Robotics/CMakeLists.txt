# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(robotics)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Use OpenCV 3
find_package(OpenCV 3 REQUIRED)

# Add the source files to build
SET(SrcFiles src/main.cpp)

# The shared library to build:
ADD_EXECUTABLE(RoboticsPart ${SrcFiles})
TARGET_LINK_LIBRARIES(RoboticsPart ${ROBWORK_LIBRARIES} ${OpenCV_LIBS})

MESSAGE(${PROJECT_NAME} " done!")
