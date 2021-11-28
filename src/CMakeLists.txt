# Source files for monoVO_KLT
file(GLOB_RECURSE monoVO_KLT_SRC_FILES
	${CMAKE_CURRENT_LIST_DIR}/${CMAKE_PROJECT_NAME}_KLT/*.cpp
	)

message(STATUS "[INFO] Compiling ${CMAKE_PROJECT_NAME}_KLT...")
message(STATUS "[INFO] --> Source files used to compile ${CMAKE_PROJECT_NAME}_KLT: ")
foreach(file ${monoVO_KLT_SRC_FILES})
	message(STATUS "[INFO]|__ ${file}")
endforeach()

message(STATUS "[INFO] 3rd party libs: ${THIRD_PARTY_LIBS}")

add_executable(${CMAKE_PROJECT_NAME}_KLT ${monoVO_KLT_SRC_FILES})
target_link_libraries(${CMAKE_PROJECT_NAME}_KLT  ${THIRD_PARTY_LIBS}) # ${OpenCV_LIBS} Sophus::Sophus) # Sophus::Sophus is crucial
add_library(${CMAKE_PROJECT_NAME}_KLT_lib STATIC ${monoVO_KLT_SRC_FILES}) # To include the code in the unit tests
message(STATUS "[INFO] Building lib ${CMAKE_PROJECT_NAME}_KLT_lib")

add_executable(evaluate_pred_traj ${CMAKE_CURRENT_LIST_DIR}/utils/evaluate_pred_traj.cpp)
target_link_libraries(evaluate_pred_traj ${THIRD_PARITY_LIBS} fmt)
message(STATUS "[INFO] Building evaluate_pred_traj")