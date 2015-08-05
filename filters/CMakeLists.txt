set(SUBSYS_NAME filters)
set(SUBSYS_DESC "Object recognition filters library")
set(SUBSYS_DEPS common)

set(build TRUE)
CORE_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
CORE_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

CORE_ADD_DOC(${SUBSYS_NAME})

if(build)
  set(srcs
    src/filter_range.cpp
    src/filter_model.cpp)

  set(incs
    include/core/${SUBSYS_NAME}/filter_range.h
    include/core/${SUBSYS_NAME}/filter_model.h)

  set(LIB_NAME core_${SUBSYS_NAME})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
  CORE_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
  target_link_libraries(${LIB_NAME} core_common ${PCL_LIBRARIES}) 
  CORE_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

  # Install include files
  CORE_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

  if(BUILD_TESTS)
    add_subdirectory(test)
  endif(BUILD_TESTS)

endif(build)
