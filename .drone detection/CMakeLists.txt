cmake_minimum_required(VERSION 3.16)
project(FastDroneDetector VERSION 1.0.0)

# Set C++ standard and optimization flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags for speed
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native -mtune=native -ffast-math")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Find OpenMP (optional)
find_package(OpenMP)

# Optional packages
find_package(PkgConfig)
if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ libzmq)
endif()

# Find or fetch nlohmann_json
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    )
    FetchContent_MakeAvailable(json)
endif()

# Include directories
include_directories(include)

# Core library sources
set(CORE_SOURCES
    src/core/fast_voxel_detector.cpp
    src/core/spatial_index.cpp
    src/core/motion_detector.cpp
    src/rtsp/stream_processor.cpp
    src/atak/cot_sender.cpp
    src/config/config_manager.cpp
    src/visualization/voxel_visualizer.cpp
)

# Create the core library
add_library(drone_detection_core STATIC ${CORE_SOURCES})
target_include_directories(drone_detection_core PUBLIC include)
target_link_libraries(drone_detection_core 
    ${OpenCV_LIBS}
    Threads::Threads
)

# Add OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(drone_detection_core OpenMP::OpenMP_CXX)
endif()

if(nlohmann_json_FOUND)
    target_link_libraries(drone_detection_core nlohmann_json::nlohmann_json)
else()
    target_link_libraries(drone_detection_core nlohmann_json)
endif()

# Add ZMQ support if available
if(ZMQ_FOUND)
    target_compile_definitions(drone_detection_core PRIVATE HAVE_ZMQ)
    target_include_directories(drone_detection_core PRIVATE ${ZMQ_INCLUDE_DIRS})
    target_link_libraries(drone_detection_core ${ZMQ_LINK_LIBRARIES})
    
    # Add zlib for compression (required when ZMQ is available)
    find_package(ZLIB REQUIRED)
    target_link_libraries(drone_detection_core ZLIB::ZLIB)
endif()

# Main executable
add_executable(fast_drone_detector src/main.cpp)
target_link_libraries(fast_drone_detector drone_detection_core)

# SIMD optimizations if available
include(CheckCXXCompilerFlag)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i386")
    # x86/x64 optimizations
    check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
    if(COMPILER_SUPPORTS_AVX2)
        target_compile_definitions(drone_detection_core PRIVATE HAVE_AVX2)
        target_compile_options(drone_detection_core PRIVATE -mavx2)
    endif()
    set(COMPILER_SUPPORTS_NEON OFF)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64")
    # ARM optimizations (Raspberry Pi, etc.)
    check_cxx_compiler_flag("-mfpu=neon" COMPILER_SUPPORTS_NEON)
    if(COMPILER_SUPPORTS_NEON)
        target_compile_definitions(drone_detection_core PRIVATE HAVE_NEON)
        target_compile_options(drone_detection_core PRIVATE -mfpu=neon)
    endif()
    set(COMPILER_SUPPORTS_AVX2 OFF)
else()
    # Other architectures - no SIMD
    set(COMPILER_SUPPORTS_AVX2 OFF)
    set(COMPILER_SUPPORTS_NEON OFF)
endif()

# Installation
install(TARGETS fast_drone_detector DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/fast_drone_detector/config)
install(DIRECTORY scripts/ DESTINATION share/fast_drone_detector/scripts)
install(FILES 
    scripts/visualize_voxel_grid.py 
    scripts/static_voxel_viewer.py 
    DESTINATION bin 
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Build configuration summary
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenMP support: ${OpenMP_CXX_FOUND}")
message(STATUS "ZMQ support: ${ZMQ_FOUND}")
message(STATUS "AVX2 support (x86): ${COMPILER_SUPPORTS_AVX2}")
message(STATUS "NEON support (ARM): ${COMPILER_SUPPORTS_NEON}") 