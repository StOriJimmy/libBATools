cmake_minimum_required (VERSION 3.0)
cmake_policy(VERSION 3.0)

project (libBATools C CXX)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

if(CMAKE_CL_64)  
    set(PLATFORM "x64")
else(CMAKE_CL_64)  
    set(PLATFORM "WIN32")
endif(CMAKE_CL_64)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${PLATFORM})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PLATFORM})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${PLATFORM})

set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_DEBUG_POSTFIX "d")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED yes)

if(WIN32)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    if(MSVC)
      string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " /NODEFAULTLIB:LIBCMT /NODEFAULTLIB:MSVCRT")
    endif()
endif()

# The version number.
set(LIBBATOOLS_VERSION_MAJOR 1)
set(LIBBATOOLS_VERSION_MINOR 0)
set(LIBBATOOLS_VERSION_PATCH 0)
set(LIBBATOOLS_VERSION
  ${LIBBATOOLS_VERSION_MAJOR}.${LIBBATOOLS_VERSION_MINOR}.${LIBBATOOLS_VERSION_PATCH})

# set path
set(DEMO_PATH ${CMAKE_SOURCE_DIR}/demo)
set(LIB_PATH ${CMAKE_SOURCE_DIR}/lib)
set(DEPEND_PATH ${CMAKE_SOURCE_DIR}/utils)

option(OPTION_BUILD_DEMO "build demo" ON)
option(OPTION_USE_VCGLIB "use vcglib" ON)
option(OPTION_BUILD_MT "use mt build" ON)
#option(OPTION_USE_BATOOLS_AS_DLL "use libBATools as dll" ON)

# set MD to MT
if(OPTION_BUILD_MT)
set(CompilerFlags
    CMAKE_CXX_FLAGS
    CMAKE_CXX_FLAGS_DEBUG
    CMAKE_CXX_FLAGS_RELEASE
    CMAKE_C_FLAGS
    CMAKE_C_FLAGS_DEBUG
    CMAKE_C_FLAGS_RELEASE
  )
foreach(CompilerFlags ${CompilerFlags})
  string(REPLACE "/MD" "/MT" ${CompilerFlags} "${${CompilerFlags}}")
endforeach()
endif()

# try to find the vcg library
if(OPTION_VCGLIB)
set (VCGLIB_INCLUDE_DIR_HINTS ${DEPEND_PATH}/vcglib)
find_package(vcglib REQUIRED)
  if (VCGLIB_FOUND)
    message(STATUS "vcglib found in:")
    message(STATUS ${VCGLIB_INCLUDE_DIRS})
    endif (VCGLIB_FOUND)
endif(OPTION_VCGLIB)

# configure a file
configure_file(${CMAKE_SOURCE_DIR}/cmake/config.h.in
  ${CMAKE_BINARY_DIR}/cmake/config.h)

include_directories(BEFORE ${CMAKE_BINARY_DIR}/cmake)
include_directories(${LIB_PATH})

if(OPTION_BUILD_DEMO)
add_subdirectory(${DEMO_PATH})
endif(OPTION_BUILD_DEMO)

add_subdirectory(${LIB_PATH})

INSTALL(TARGETS libBATools
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION libstatic
)