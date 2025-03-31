# Minimum required version of CMake
cmake_minimum_required(VERSION 3.0)

# Project name and language
project(stupidSerializer)

# Set the library source files
set(SOURCES
    include/stupidSerializer.h
)

# Add the library target
# Use STATIC for a static library, SHARED for a shared library
add_library(stupidSerializer STATIC ${SOURCES})

# Specify the include directory for this library
target_include_directories(stupidSerializer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Optionally set the C++ standard (e.g., C20)
set_target_properties(stupidSerializer PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)