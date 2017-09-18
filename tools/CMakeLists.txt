set(SUBSYS_NAME tools)
set(SUBSYS_DESC "Useful CORE-based command line tools")
set(SUBSYS_DEPS common configuration utils filters segmentation features descriptors learning classification)

set(build TRUE)
CORE_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
CORE_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

CORE_ADD_DOC(${SUBSYS_NAME})

if(build)
  include_directories(${CMAKE_SOURCE_DIR}/3rdparty/libsvm)

  CORE_ADD_EXECUTABLE(compute_covariance ${SUBSYS_NAME} descriptors/covariance/compute_covariance.cpp)
  target_link_libraries(compute_covariance core_configuration core_descriptors)

  CORE_ADD_EXECUTABLE(svm_learn_model ${SUBSYS_NAME} learning/svm/svm_learn_model.cpp)
  target_link_libraries(svm_learn_model core_configuration core_learning)

  CORE_ADD_EXECUTABLE(svm_predict_class ${SUBSYS_NAME} classification/svm/svm_predict_class.cpp)
  target_link_libraries(svm_predict_class core_configuration core_classification)

endif(build)
