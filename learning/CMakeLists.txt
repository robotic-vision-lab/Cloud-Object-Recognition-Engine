set(SUBSYS_NAME learning)
set(SUBSYS_DESC "Object recognition learning library")
set(SUBSYS_DEPS common)

set(build TRUE)
CORE_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
CORE_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

CORE_ADD_DOC(${SUBSYS_NAME})

if(build)
  set(srcs 
    src/learn_svm_covariance_model.cpp
    src/svm.cpp)

  set(incs 
    include/core/${SUBSYS_NAME}/learn_svm_covariance_model.h
    include/svm.h)

  CORE_ADD_EXECUTABLE(learn_svm_covariance_model ${SUBSYS_NAME} 
                     src/learn_svm_covariance_model.cpp
                     src/svm.cpp)
  target_link_libraries(learn_svm_covariance_model core_common ${PCL_LIBRARIES})
  

  set(LIB_NAME core_${SUBSYS_NAME})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
  CORE_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
  target_link_libraries(${LIB_NAME} core_common)
  CORE_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

  # Install include files
  CORE_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})

  if(BUILD_TESTS)
    add_subdirectory(test)
  endif(BUILD_TESTS)

endif(build)
