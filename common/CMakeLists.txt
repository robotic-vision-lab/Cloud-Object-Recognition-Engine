set(SUBSYS_NAME common)
set(SUBSYS_DESC "Object recognition common library")
set(SUBSYS_DEPS)

set(build TRUE)
CORE_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
CORE_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

CORE_ADD_DOC(${SUBSYS_NAME})

if(build)
  set(srcs 
    src/parse.cpp
    src/print.cpp)

  set(incs 
    include/core/core_macros.h)
        
  set(tools_incs 
    include/core/console/parse.h
    include/core/console/print.h
    include/core/console/time.h)
        
  set(LIB_NAME core_${SUBSYS_NAME})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
  CORE_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
  CORE_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

  # Install include files
  CORE_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})
  CORE_ADD_INCLUDES(${SUBSYS_NAME} console ${tools_incs})

  if(BUILD_TESTS)
     add_subdirectory(test)
  endif(BUILD_TESTS)

endif(build)
