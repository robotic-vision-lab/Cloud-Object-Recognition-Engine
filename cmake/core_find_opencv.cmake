# Find and set OpenCV flags

find_package(OpenCV REQUIRED)

if(OPENCV_FOUND)
  add_definitions(${OpenCV_DEFINITIONS})
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
endif(OPENCV_FOUND)
