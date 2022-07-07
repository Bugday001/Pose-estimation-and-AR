cmake_minimum_required(VERSION 3.9)
project(camera_calibration)

set(Ceres_DIR "C:/Program Files/Ceres/bin")
message("-- Check for Ceres solver")
find_package(Ceres REQUIRED)
if(CERES_FOUND)
	message("-- Found Ceres Solver: ${CERES_INCLUDE_DIRS}")
else(CERES_FOUND)
	message(FATAL_ERROR "Can't find Ceres Solver. Please set CERES_INCLUDE_DIR & CERES_LIBRARY")
endif(CERES_FOUND)
  
include_directories(${CMAKE_INCLUDE_PATH} ${CERES_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(modules)
# Add the executable
add_executable(camera_calibrator camera_calibrator.cpp)
target_link_libraries(camera_calibrator ${CERES_LIBRARIES})
target_link_libraries(camera_calibrator ${OpenCV_LIBS})



