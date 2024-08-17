cmake_minimum_required(VERSION 3.4.1)

set(PROJ_NAME Actiger_controller)
project(${PROJ_NAME})

# 加入一个配置头文件，用于处理 CMake 对源码的设置
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_SOURCE_DIR}/config.h"
  )
 
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O3")

# 是否使用实际机器人
option (ROBOT "Use Real Robot" OFF)

# 打印消息
MESSAGE(STATUS "Use Real Robot = ${ROBOT}")

# 包含所有.cpp 和.hpp
FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/controller/legController/*.hpp" 
		                  "${CMAKE_SOURCE_DIR}/controller/model/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/bodyTrajectory/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/endfootTrajectory/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/foothold/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/terrain/*.hpp" 
		                  "${CMAKE_SOURCE_DIR}/controller/motionPlan/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/mpc/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/module/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/qpSolver/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/WBIC/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/taskSet/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/contact/*.hpp"
                      "${CMAKE_SOURCE_DIR}/controller/minisnap/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/*.hpp" 
                      "${CMAKE_SOURCE_DIR}/*.h"
		                  "${CMAKE_SOURCE_DIR}/embedded/*.h"
		                  "${CMAKE_SOURCE_DIR}/embedded/*.hpp"
                      "${CMAKE_SOURCE_DIR}/webots/*.hpp"
                      "${CMAKE_SOURCE_DIR}/modules/*.hpp")

FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/controller/legController/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/model/*.cpp"
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/bodyTrajectory/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/endfootTrajectory/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/foothold/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/terrain/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/motionPlan/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/mpc/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/module/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/qpSolver/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/WBIC/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/taskSet/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/wbc/contact/*.cpp" 
                      "${CMAKE_SOURCE_DIR}/controller/minisnap/*.cpp"
                      "${CMAKE_SOURCE_DIR}/*.cpp"
		                  "${CMAKE_SOURCE_DIR}/embedded/*.c"
		                  "${CMAKE_SOURCE_DIR}/embedded/*.cpp"
                      "${CMAKE_SOURCE_DIR}/webots/*.cpp"
                      "${CMAKE_SOURCE_DIR}/modules/*.cpp")

# 设置输出
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})
add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST})

include_directories(${PROCESS_COMM_DIR}
                    ${CMAKE_SOURCE_DIR}/controller/legController
                    ${CMAKE_SOURCE_DIR}/controller/model
                    ${CMAKE_SOURCE_DIR}/controller/motionPlan/bodyTrajectory
                    ${CMAKE_SOURCE_DIR}/controller/motionPlan/endfootTrajectory
                    ${CMAKE_SOURCE_DIR}/controller/motionPlan/foothold
                    ${CMAKE_SOURCE_DIR}/controller/motionPlan/terrain
                    ${CMAKE_SOURCE_DIR}/controller/motionPlan
                    ${CMAKE_SOURCE_DIR}/controller/mpc
                    ${CMAKE_SOURCE_DIR}/controller/module
                    ${CMAKE_SOURCE_DIR}/controller/qpSolver
                    ${CMAKE_SOURCE_DIR}/controller/wbc
                    ${CMAKE_SOURCE_DIR}/controller/wbc/WBIC
                    ${CMAKE_SOURCE_DIR}/controller/wbc/taskSet
                    ${CMAKE_SOURCE_DIR}/controller/wbc/contact
                    ${CMAKE_SOURCE_DIR}/controller/minisnap
                    ${CMAKE_SOURCE_DIR}/embedded
                    ${CMAKE_SOURCE_DIR}/webots
                    ${CMAKE_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/modules) 

set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(OSQP_LIBRARY_DIRS /usr/local/lib)
set(RBDL_LIBRARY_DIRS /usr/local/lib)  

set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
set(OSQP_INCLUDE_DIRS /usr/local/include/osqp)
set(RBDL_INCLUDE_DIRS /usr/local/include/rbdl)

include_directories(${EIGEN_INCLUDE_DIRS}) 
include_directories(${IPOPT_INCLUDE_DIRS})
include_directories(${OSQP_INCLUDE_DIRS})
include_directories(${RBDL_INCLUDE_DIRS} )

link_directories(${OSQP_LIBRARY_DIRS})
target_link_libraries(${PROJ_NAME} osqp)

link_directories(${IPOPT_LIBRARY_DIRS})
target_link_libraries(${PROJ_NAME} ipopt)


link_directories(${RBDL_LIBRARY_DIRS})
target_link_libraries(${PROJ_NAME}  rbdl)

link_directories(${RBDL_LIBRARY_DIRS})
target_link_libraries(${PROJ_NAME}  rbdl_urdfreader)


# 是否需要引入webots
if (ROBOT) 
  target_link_libraries(${PROJ_NAME} /home/bean/code/Actiger_controller/embedded/lib/libGinkgo_Driver.so)
  target_link_libraries(${PROJ_NAME} /home/bean/code/Actiger_controller/embedded/lib/libusb.so)
  target_link_libraries(${PROJ_NAME} /home/bean/code/Actiger_controller/embedded/lib/libusb-1.0.so)
else ()
  set(WEBOTS_LIBRARY_DIRS /usr/local/webots/lib/controller)
  
  set(WEBOTS_CPP_INCLUDE_DIRS /usr/local/webots/include/controller/cpp)
  set(WEBOTS_C_INCLUDE_DIRS /usr/local/webots/include/controller/c)

  include_directories(${WEBOTS_CPP_INCLUDE_DIRS}) 
  include_directories(${WEBOTS_C_INCLUDE_DIRS}) 

  target_link_libraries(${PROJ_NAME} /usr/local/webots/lib/controller/libCppController.so)
  target_link_libraries(${PROJ_NAME} /usr/local/webots/lib/controller/libController.so)
endif ()
