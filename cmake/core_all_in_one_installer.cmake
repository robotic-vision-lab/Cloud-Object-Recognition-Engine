if(WIN32)
    option(BUILD_all_in_one_installer "Build an all-in-one NSIS installer" OFF)
endif(WIN32)

if(BUILD_all_in_one_installer)
  get_filename_component(BOOST_ROOT "${Boost_INCLUDE_DIR}" PATH)
  get_filename_component(EIGEN_ROOT "${EIGEN_INCLUDE_DIRS}" PATH)
  set(CORE_3RDPARTY_COMPONENTS)
  foreach(dep Eigen Boost)
    string(TOUPPER ${dep} DEP)
    install(DIRECTORY "${${DEP}_ROOT}/"
            DESTINATION 3rdParty/${dep}
            COMPONENT ${dep}
            PATTERN "*/Uninstall.exe" EXCLUDE
            )
    list(APPEND CORE_3RDPARTY_COMPONENTS ${dep})
  endforeach(dep)
  #list(REMOVE_DUPLICATES CORE_3RDPARTY_COMPONENTS)
endif(BUILD_all_in_one_installer)
