cmake_minimum_required(VERSION 2.8.3)
project(knowledge)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  #baxter_core_msgs
  message_generation
  std_msgs
	pitt_msgs
	knowledge_msgs
  actionlib
)
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
	include/
)



#Is for add the .o files to devel/lib

catkin_package(	INCLUDE_DIRS include CATKIN_DEPENDS message_generation roscpp rospy std_msgs geometry_msgs  knowledge_msgs pitt_msgs)# baxter_core_msgs 


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


#ADD_LIBRARY(endor /home/nasa/Documents/Endor/endor-master/aograph.cpp)
add_library(Objects include/Objects.cpp)
#add_library(Points include/Points.hpp)
#add_library(World include/World.cpp)
#add_library(endorAction src/endorAction.cpp)


## Declare a C++ executable
# name  name.cpp
add_executable(knowledge src/knowledge_main.cpp )

add_dependencies(knowledge geometry_msgs_generate_messages_cpp knowledge_msgs_generate_messages_cpp pitt_msgs_generate_messages_cpp) #baxter_core_msgs_generate_messages_cpp

add_dependencies(Objects geometry_msgs_generate_messages_cpp knowledge_msgs_generate_messages_cpp pitt_msgs_generate_messages_cpp)#baxter_core_msgs_generate_messages_cpp

## Specify libraries to link a library or executable target against
target_link_libraries(knowledge
   ${catkin_LIBRARIES}
Objects
)

catkin_install_python(PROGRAMS scripts/pitt_runner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
