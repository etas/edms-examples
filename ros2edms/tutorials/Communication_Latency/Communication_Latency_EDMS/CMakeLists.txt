cmake_minimum_required(VERSION 3.16)
project(producer_consumer)

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_MODULE_PATH})

find_package(aos_build_tools REQUIRED)
find_package(aos_runtime_prod REQUIRED)
find_package(aos_runtime_dev REQUIRED)

include(yaaa_init)
yaaa_init(
  ROOT_FILE "deployment/DeploymentLinux.deploy.yaml"
  YAAA_IMPORT_DIRS
    deployment
    activities
    runnables
    interfaces
)

add_subdirectory(interfaces)
add_subdirectory(runnables)
add_subdirectory(activities)
add_subdirectory(deployment)
