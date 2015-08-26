set(SUBSYS_NAME classification)
set(SUBSYS_DESC "Object recognition classification library")
set(SUBSYS_DEPS common utils filters segmentation features descriptors)

set(build TRUE)
CORE_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
CORE_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

CORE_ADD_DOC(${SUBSYS_NAME})

if(build)
  set(srcs
      src/svm/covariance_predict_class.cpp
      ${CMAKE_SOURCE_DIR}/3rdparty/libsvm/svm.cpp) 

  set(incs 
      include/core/${SUBSYS_NAME}/svm/covariance_predict_class.h)

  set(LIB_NAME core_${SUBSYS_NAME})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                      ${CMAKE_SOURCE_DIR}/3rdparty/libsvm
                      ${PCL_INCLUDE_DIRS})
  CORE_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
  target_link_libraries(${LIB_NAME} 
                        core_common 
                        core_utils
                        core_filters 
                        core_segmentation
                        core_features 
                        core_descriptors)
  CORE_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" "" "" "" "")

  # Install include files
  CORE_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})

  if(BUILD_TESTS)
    add_subdirectory(test)
  endif(BUILD_TESTS)

endif(build)
