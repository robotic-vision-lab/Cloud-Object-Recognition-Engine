# Find and set PCL flags

find_package(PCL REQUIRED)

if(PCL_FOUND)
  add_definitions(${PCL_DEFINITIONS})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
endif(PCL_FOUND)
