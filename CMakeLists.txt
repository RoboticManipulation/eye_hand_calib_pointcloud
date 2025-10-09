cmake_minimum_required(VERSION 3.16)
project(RegHEC LANGUAGES CXX)

if(UNIX)
  add_compile_options(-UWIN32 -U_WIN32 -U_MSC_VER -UWIN64 -U_WIN64)
endif()

# ---- Build settings ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Helpful warnings (optional)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unknown-pragmas)
endif()

# ---- Dependencies ----
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree visualization filters)
find_package(Boost REQUIRED)

# Sophus (header-only)
find_package(Sophus QUIET)
if (Sophus_FOUND)
  message(STATUS "Found Sophus: ${Sophus_DIR}")
else()
  message(STATUS "Sophus not found via find_package, will try standard paths")
endif()

# --- NLopt (handle multiple package/target names) ---
find_package(NLOPT QUIET)
if(NOT NLOPT_FOUND)
  find_package(NLopt QUIET)
endif()

set(NLOPT_TARGET "")
if (TARGET NLOPT::nlopt)
  set(NLOPT_TARGET NLOPT::nlopt)
elseif (TARGET NLopt::nlopt)
  set(NLOPT_TARGET NLopt::nlopt)
elseif (DEFINED NLOPT_LIBRARIES)
  set(NLOPT_TARGET ${NLOPT_LIBRARIES})
else()
  find_library(NLOPT_TARGET nlopt)
endif()

if (NOT NLOPT_TARGET)
  message(FATAL_ERROR "Could not find NLopt library. Install libnlopt-dev or set NLOPT_DIR/NLopt_DIR.")
endif()

# Collect NLopt include directories
set(NLOPT_INCLUDE_DIRS_CAND "")
if (DEFINED NLOPT_INCLUDE_DIRS)
  list(APPEND NLOPT_INCLUDE_DIRS_CAND ${NLOPT_INCLUDE_DIRS})
endif()
if (DEFINED NLopt_INCLUDE_DIRS)
  list(APPEND NLOPT_INCLUDE_DIRS_CAND ${NLopt_INCLUDE_DIRS})
endif()
get_target_property(_nlopt_iface_inc ${NLOPT_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
if (_nlopt_iface_inc)
  list(APPEND NLOPT_INCLUDE_DIRS_CAND ${_nlopt_iface_inc})
endif()
if (NLOPT_INCLUDE_DIRS_CAND STREQUAL "")
  list(APPEND NLOPT_INCLUDE_DIRS_CAND /usr/include)
endif()
list(REMOVE_DUPLICATES NLOPT_INCLUDE_DIRS_CAND)

# ---- Limbo ----
# Try to find Limbo in standard locations or via LIMBO_DIR environment variable
if(DEFINED ENV{LIMBO_DIR})
    set(LIMBO_INCLUDE_DIR $ENV{LIMBO_DIR}/src)
    message(STATUS "Using Limbo from LIMBO_DIR: ${LIMBO_INCLUDE_DIR}")
else()
    # Try standard installation paths
    find_path(LIMBO_INCLUDE_DIR limbo/limbo.hpp
        PATHS 
            /usr/local/include
            /usr/include
            ${CMAKE_SOURCE_DIR}/include  # For bundled custom headers
        DOC "Limbo include directory")
    
    if(LIMBO_INCLUDE_DIR)
        message(STATUS "Found Limbo at: ${LIMBO_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Limbo not found. Please:\n"
                "  1. Install limbo: git clone https://github.com/resibots/limbo.git\n"
                "  2. Set LIMBO_DIR environment variable: export LIMBO_DIR=/path/to/limbo\n"
                "  Or place limbo in /usr/local/include")
    endif()
endif()

# ---- Target ----
add_executable(reghec
  src/RegHEC_multi.cpp
  src/GNsolver.cpp
)

target_compile_definitions(reghec PRIVATE USE_NLOPT)
add_definitions(${PCL_DEFINITIONS})

target_include_directories(reghec PRIVATE
  ${CMAKE_SOURCE_DIR}/include  # For custom expSE3.hpp
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIMBO_INCLUDE_DIR}
  ${NLOPT_INCLUDE_DIRS_CAND}
)

# Sophus include
if (Sophus_FOUND)
  target_include_directories(reghec PRIVATE ${Sophus_INCLUDE_DIRS})
else()
  target_include_directories(reghec PRIVATE /usr/local/include /usr/include)
endif()

target_link_libraries(reghec PRIVATE
  ${PCL_LIBRARIES}
  ${NLOPT_TARGET}
)

find_package(Threads QUIET)
if (Threads_FOUND)
  target_link_libraries(reghec PRIVATE Threads::Threads)
endif()

install(TARGETS reghec RUNTIME DESTINATION bin)
