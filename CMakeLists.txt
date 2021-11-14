cmake_minimum_required(VERSION 2.8)
project(monoVO)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB monoVO_KLT_SRC_FILES
	${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}_KLT/*.cpp
	)

# Include all subfolders of include.
# To include a specific header, use:
# #include "<subfolder><filename>.hpp"
include_directories(${PROJECT_SOURCE_DIR}/include)

message(STATUS "[INFO] Compiling ${PROJECT_NAME}_KLT...")
message(STATUS "[INFO] --> Source files used to compile ${PROJECT_NAME}_KLT: ")

foreach(file ${monoVO_KLT_SRC_FILES})
	message(STATUS "[INFO]|__ ${file}")
endforeach()

add_executable(${PROJECT_NAME}_KLT ${monoVO_KLT_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_KLT ${OpenCV_LIBS})
