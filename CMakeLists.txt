cmake_minimum_required (VERSION 2.6)
project(MVPS)

# Project configuration
set(VERSION_PRJ 0.1)

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
elseif(APPLE)
    add_definitions(-Wno-narrowing -pthread)
    add_definitions(-std=gnu++11 -stdlib=libc++)
endif()

######################################
########## Build libraries ###########
######################################
MESSAGE(STATUS "--------------------------------------------------------------------------------")
MESSAGE(STATUS "-------------------- Preparing GPIS SLAM Library -------------------------------")
MESSAGE(STATUS "--------------------------------------------------------------------------------")
add_subdirectory(dep/cjson)

MESSAGE(STATUS "--------------------------------------------------------------------------------")
MESSAGE(STATUS "------------------- Preparing RGB-D SLAM Library -------------------------------")
MESSAGE(STATUS "--------------------------------------------------------------------------------")
add_subdirectory(dep/rgbd_slam)

######################################
########## Build projects ############
######################################

MESSAGE(STATUS "--------------------------------------------------------------------------------")
MESSAGE(STATUS "----------------------- Preparing Registration ---------------------------------")
MESSAGE(STATUS "--------------------------------------------------------------------------------")
add_subdirectory(ObjectScanner)


######################################
######## Organize projects ###########
######################################
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set_property(TARGET rgbdSlam PROPERTY FOLDER "dep")
set_property(TARGET cjson PROPERTY FOLDER "dep")

