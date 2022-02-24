cmake_minimum_required(VERSION 3.14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(DownloadProject)

# ============================================================
#                        cppbase
# ============================================================
#
message("==> Configuring cppbase")
set(CPPBASE_BUILD_GTEST OFF)
download_project(
  PROJ     cppbase
  URL      ${CMAKE_CURRENT_SOURCE_DIR}/cppbase/cppbase.tar.gz
  URL_HASH MD5=cc46d3d7ceba5cde9127d03fc4a76446
  QUIET
)
add_subdirectory(${cppbase_SOURCE_DIR} ${cppbase_BINARY_DIR})
set(CPPBASE_INCLUDE_DIR ${cppbase_SOURCE_DIR} CACHE STRING "")

# ============================================================
#                        lidardevice
# ============================================================
#
message("==> Configuring lidardevice")
download_project(
  PROJ     lidardevice
  URL      ${CMAKE_CURRENT_SOURCE_DIR}/lidardevice/LidarDevice-1.0.0-Linux.tar.gz
  URL_HASH MD5=20c9be328eb3c028fb6f8d5dd1d73636
  QUIET
)
add_library(lidardevice INTERFACE)
target_include_directories(lidardevice INTERFACE ${lidardevice_SOURCE_DIR}/include)
target_link_directories(lidardevice INTERFACE ${lidardevice_SOURCE_DIR}/lib)
target_link_libraries(lidardevice INTERFACE LidarDevice)
set(lidardevice_SOURCE_DIR ${lidardevice_SOURCE_DIR} CACHE STRING "")
