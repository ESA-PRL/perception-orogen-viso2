PROJECT(viso2)
cmake_minimum_required(VERSION 2.6)

add_definitions(-DWall)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
add_definitions(-DBASE_LOG_DISABLE)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(viso2Base)
