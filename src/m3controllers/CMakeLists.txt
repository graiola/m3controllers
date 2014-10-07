###############################################################################
# Project name is not mandatory, but you should use it
# project(m3controllers)

###############################################################################
# General settings
# Some settings related to building/installing libraries and header files
# Set this to STATIC if you want to build static libraries instead.
set(SHARED_OR_STATIC "SHARED")
# Some flags for CXX
#set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -DTIMING")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

###############################################################################
# Find RTAI
find_package(RTAI REQUIRED)
set(LIBRARIES ${LIBRARIES} ${RTAI_LIBRARY})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${RTAI_INCLUDE_DIR})

###############################################################################
# Find M3
find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3_INCLUDE_DIR})

###############################################################################
# Find M3RT
find_package(M3rt REQUIRED)
set(LIBRARIES ${LIBRARIES} ${M3RT_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${M3RT_INCLUDE_DIR})

###############################################################################
# Find google protobuff
find_package(Protobuf REQUIRED)
#set(PROTOBUF_IMPORT_DIRS ${INCLUDE_DIRS} ${M3ENS_PROTO_DIR}) #NOTE I am able to import .proto using m3ens/m3_controllers... but then there is a problem with the .h inclusion
set(PROTOBUF_IMPORT_DIRS ${INCLUDE_DIRS})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})

###############################################################################
# Find Eigen
find_package(Eigen3 3.2 REQUIRED)
set(LIBRARIES ${LIBRARIES} ${EIGEN3_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

###############################################################################
# Find boost packages
find_package(Boost 1.34 COMPONENTS filesystem system serialization REQUIRED)
set(LIBRARIES ${LIBRARIES} ${Boost_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

###############################################################################
# Find Catkin components (optionals)
find_package(catkin COMPONENTS kdl_kinematics virtual_mechanism sensor_msgs geometry_msgs nav_msgs realtime_tools)
if(realtime_tools_FOUND AND sensor_msgs_FOUND)
   add_definitions(-DUSE_ROS_RT_PUBLISHER)
endif()
set(LIBRARIES ${LIBRARIES} ${catkin_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

###############################################################################
# Find Yamlcpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
if(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
find_package(YamlCpp  REQUIRED)
set_property(
   DIRECTORY
   PROPERTY COMPILE_DEFINITIONS YAMLCPP_05
   )
endif(NOT ${YAMLCPP_VERSION} VERSION_LESS "0.5")
set(LIBRARIES ${LIBRARIES} ${YAMLCPP_LIBRARIES})
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIR})
#link_directories(${YAMLCPP_LIBRARY_DIRS})

###############################################################################
# Find DMP
set(DMP_LIBRARIES dmp dynamicalsystems functionapproximators)
set(LIBRARIES ${LIBRARIES} ${DMP_LIBRARIES})

###############################################################################
# Include directories
include_directories(${INCLUDE_DIRS})

###############################################################################
# Things to compile are in their own directories
add_subdirectory(m3_controller_interface)
add_subdirectory(m3_controller_manager)
add_subdirectory(sin_controller)
add_subdirectory(joints_controller)
#add_subdirectory(torque_controller)
#add_subdirectory(vel_controller)
if(kdl_kinematics_FOUND)
        add_subdirectory(cart_controller)
        #add_subdirectory(dmp_controller) #NOTE no way to check if dmpboo is installed...
        #add_subdirectory(vf_controller)
        if(virtual_mechanism_FOUND)
	     add_subdirectory(vf_force_controller)
	endif()
endif()

###############################################################################
# Create the m3controllers library, to be loaded in the m3_rt_factory
add_library(${PROJECT_NAME} SHARED factory_proxy.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS "${FACTORY_DEFINITIONS}")

###############################################################################
# Linking m3controllers
target_link_libraries(${PROJECT_NAME} ${LIBNAME_LIST})