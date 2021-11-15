set(BINARY ${CMAKE_PROJECT_NAME}_KLT_tst)
file(GLOB_RECURSE TEST_SOURCES *.hpp *.cpp)

message(STATUS "TEST_SOURCES: ${TEST_SOURCES}")

add_executable(${BINARY} ${TEST_SOURCES})
add_test(NAME ${BINARY} COMMAND ${BINARY})
# Link against 3rd party libraries, gtest, and the static lib built by cmake
target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME}_KLT_lib ${THIRD_PARTY_LIBS} gtest) # THIRD_PARTY_LIBS is set in the top-level CMakeLists.txt
