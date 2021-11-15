cmake_minimum_required(VERSION 2.8)
project(monoVO)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Locate dependency packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(src) # Add sources

# Include all subfolders of include.
# To include a specific header, use:
# #include "<subfolder><filename>.hpp"
include_directories(${PROJECT_SOURCE_DIR}/include)

# Set target library names
set(THIRD_PARTY_LIBS ${OpenCV_LIBS} Sophus::Sophus Eigen3::Eigen)

# Build the main target
add_subdirectory(src)

# Add tests
option(TESTING "Enable testing" ON) # To build tests, run cmake with -DTESTING=ON (default OFF)
if (${TESTING})
	# Add googletest
	message(STATUS "[INFO] Compiling tests...")
	enable_testing()
	add_subdirectory(tests)
	add_subdirectory(lib/googletest)
endif()