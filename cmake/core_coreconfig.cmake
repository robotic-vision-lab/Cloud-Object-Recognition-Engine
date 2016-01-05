set(CORE_SUBSYSTEMS_MODULES ${CORE_SUBSYSTEMS})
#list(REMOVE_ITEM CORE_SUBSYSTEMS_MODULES tools cuda_apps global_tests proctor examples)

set(CORECONFIG_AVAILABLE_COMPONENTS)
set(CORECONFIG_AVAILABLE_COMPONENTS_LIST)
set(CORECONFIG_INTERNAL_DEPENDENCIES)
set(CORECONFIG_EXTERNAL_DEPENDENCIES)
set(CORECONFIG_OPTIONAL_DEPENDENCIES)
foreach(_ss ${CORE_SUBSYSTEMS_MODULES})
  CORE_GET_SUBSYS_STATUS(_status ${_ss})
  if(_status)
    set(CORECONFIG_AVAILABLE_COMPONENTS "${CORECONFIG_AVAILABLE_COMPONENTS} ${_ss}")
    set(CORECONFIG_AVAILABLE_COMPONENTS_LIST "${CORECONFIG_AVAILABLE_COMPONENTS_LIST}\n# - ${_ss}")
    GET_IN_MAP(_deps CORE_SUBSYS_DEPS ${_ss})
    if(_deps)
      set(CORECONFIG_INTERNAL_DEPENDENCIES "${CORECONFIG_INTERNAL_DEPENDENCIES}set(core_${_ss}_int_dep ")
      foreach(_dep ${_deps})
        set(CORECONFIG_INTERNAL_DEPENDENCIES "${CORECONFIG_INTERNAL_DEPENDENCIES}${_dep} ")
      endforeach(_dep)
      set(CORECONFIG_INTERNAL_DEPENDENCIES "${CORECONFIG_INTERNAL_DEPENDENCIES})\n")
    endif(_deps)
    GET_IN_MAP(_ext_deps CORE_SUBSYS_EXT_DEPS ${_ss})
    if(_ext_deps)
      set(CORECONFIG_EXTERNAL_DEPENDENCIES "${CORECONFIG_EXTERNAL_DEPENDENCIES}set(core_${_ss}_ext_dep ")
      foreach(_ext_dep ${_ext_deps})
        set(CORECONFIG_EXTERNAL_DEPENDENCIES "${CORECONFIG_EXTERNAL_DEPENDENCIES}${_ext_dep} ")
      endforeach(_ext_dep)
      set(CORECONFIG_EXTERNAL_DEPENDENCIES "${CORECONFIG_EXTERNAL_DEPENDENCIES})\n")
    endif(_ext_deps)	
    GET_IN_MAP(_opt_deps CORE_SUBSYS_OPT_DEPS ${_ss})
    if(_opt_deps)
      set(CORECONFIG_OPTIONAL_DEPENDENCIES "${CORECONFIG_OPTIONAL_DEPENDENCIES}set(core_${_ss}_opt_dep ")
      foreach(_opt_dep ${_opt_deps})
        set(CORECONFIG_OPTIONAL_DEPENDENCIES "${CORECONFIG_OPTIONAL_DEPENDENCIES}${_opt_dep} ")
      endforeach(_opt_dep)
      set(CORECONFIG_OPTIONAL_DEPENDENCIES "${CORECONFIG_OPTIONAL_DEPENDENCIES})\n")
    endif(_opt_deps)
  endif(_status)
endforeach(_ss)

configure_file("${CORE_SOURCE_DIR}/COREConfig.cmake.in"
               "${CORE_BINARY_DIR}/COREConfig.cmake" @ONLY)
configure_file("${CORE_SOURCE_DIR}/COREConfigVersion.cmake.in"
               "${CORE_BINARY_DIR}/COREConfigVersion.cmake" @ONLY)
install(FILES
        "${CORE_BINARY_DIR}/COREConfig.cmake"
        "${CORE_BINARY_DIR}/COREConfigVersion.cmake"
        COMPONENT coreconfig
        DESTINATION ${CORECONFIG_INSTALL_DIR})