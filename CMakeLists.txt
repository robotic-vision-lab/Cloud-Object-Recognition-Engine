### ---[ CORE global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(CORE)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()
if(WIN32)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "_debug")
  endif()
  if(NOT DEFINED CMAKE_RELEASE_POSTFIX)
    set(CMAKE_RELEASE_POSTFIX "_release")
  endif()
endif()

# ---[ special maintainer mode
SET(CMAKE_CXX_FLAGS_MAINTAINER "-pedantic -Wno-variadic-macros -Wabi -Wextra -Weffc++" CACHE STRING
    "Flags used by the C++ compiler during maintainer builds."
    FORCE)
SET(CMAKE_C_FLAGS_MAINTAINER "-pedantic -Wno-variadic-macros -Wabi -Wextra -Weffc++" CACHE STRING
    "Flags used by the C compiler during maintainer builds."
    FORCE)
SET(CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used for linking binaries during maintainer builds."
    FORCE)
SET(CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used by the shared libraries linker during maintainer builds."
    FORCE)
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_MAINTAINER
    CMAKE_C_FLAGS_MAINTAINER
    CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    CMAKE_SHARED_LINKER_FLAGS_MAINTAINER)

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

# ---[ Android check
if(ANDROID_NDK)
  set(CORE_SHARED_LIBS OFF)
  message("CORE shared libs on Android must be: ${CORE_SHARED_LIBS}")
endif()

include(${CORE_SOURCE_DIR}/cmake/core_targets.cmake)
include(${CORE_SOURCE_DIR}/cmake/core_options.cmake)

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unknown-pragmas -fno-strict-aliasing -pthread")
  if(WIN32)
    if(CORE_SHARED_LIBS)
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-all-symbols -Wl,--enable-auto-import")
    else(CORE_SHARED_LIBS)
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif(CORE_SHARED_LIBS)
  endif()
endif()

if(MSVC)
  add_definitions ("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /EHsc /fp:fast /wd4800 /wd4521 /wd4251 /wd4275")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL /wd4800")
  SET(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
  SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
  if(MSVC90 OR MSVC10)
    option(ENABLE_MULTI_PROCESSOR_COMPILATION "Enable multi-processor compilation." ON)
    mark_as_advanced(ENABLE_MULTI_PROCESSOR_COMPILATION)
    if(ENABLE_MULTI_PROCESSOR_COMPILATION)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    endif(ENABLE_MULTI_PROCESSOR_COMPILATION)
  endif()
endif()

if (__COMPILER_PATHSCALE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-uninitialized -zerouv -pthread -mp")
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

# ---[ Project folders
option(USE_PROJECT_FOLDERS "Use folders to organize CORE projects in an IDE." OFF)
mark_as_advanced(USE_PROJECT_FOLDERS)
if(USE_PROJECT_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif(USE_PROJECT_FOLDERS)

include(${CORE_SOURCE_DIR}/cmake/core_utils.cmake)
set(CORE_VERSION 0.1.0 CACHE STRING "CORE version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

set(CORE_OUTPUT_LIB_DIR ${CORE_BINARY_DIR}/${LIB_INSTALL_DIR})
set(CORE_OUTPUT_BIN_DIR ${CORE_BINARY_DIR}/${BIN_INSTALL_DIR})
make_directory(${CORE_OUTPUT_LIB_DIR})
make_directory(${CORE_OUTPUT_BIN_DIR})
if(WIN32)
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${CORE_OUTPUT_LIB_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${CORE_OUTPUT_BIN_DIR}")
    # ---[ Windows requires DLLs (shared libraries) to be installed in the same directory as executables
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${CORE_OUTPUT_BIN_DIR}")
  endforeach(config)
else(WIN32)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CORE_OUTPUT_LIB_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CORE_OUTPUT_BIN_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CORE_OUTPUT_LIB_DIR}")
endif(WIN32)

# Add an "uninstall" target
configure_file("${CORE_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
               "${CORE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${CORE_BINARY_DIR}/uninstall_target.cmake")

#
# This is copy/paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
#
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" is_system_dir)
if("${is_system_dir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
endif("${is_system_dir}" STREQUAL "-1")

### ---[ Find universal dependencies
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(HAVE_OPENMP 1)
  message(STATUS "HAVE_OPENMP status ${HAVE_OPENMP}")
  if(MSVC90 OR MSVC10)
    if(MSVC90)
      set(OPENMP_DLL VCOMP90)
    elseif(MSVC10)
      set(OPENMP_DLL VCOMP100)
    endif(MSVC90)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
  endif(MSVC90 OR MSVC10)
else(OPENMP_FOUND)
  message (STATUS "Not found OpenMP")
endif()

# Boost (required)
include(${CORE_SOURCE_DIR}/cmake/core_find_boost.cmake)

# Eigen (required)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

# PCL (required)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV (required)
include(${CORE_SOURCE_DIR}/cmake/core_find_opencv.cmake)

# Find Doxygen and html help compiler if any
find_package(Doxygen)
if(DOXYGEN_FOUND)
  find_package(HTMLHelp)
endif(DOXYGEN_FOUND)

### ---[ Create the config.h file
set(core_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/core_config.h.in")
set(core_config_h "${CMAKE_CURRENT_BINARY_DIR}/include/core/core_config.h")
configure_file(${core_config_h_in} ${core_config_h})
CORE_ADD_INCLUDES(common "" ${core_config_h})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

### ---[ Set up for tests
#include(${CORE_SOURCE_DIR}/cmake/core_tests.cmake)

### ---[ Set up for examples
#include(${CORE_SOURCE_DIR}/cmake/core_examples.cmake)

### ---[ Add the libraries subdirectories
include(${CORE_SOURCE_DIR}/cmake/core_targets.cmake)

collect_subproject_directory_names(${CORE_SOURCE_DIR} "CMakeLists.txt" CORE_MODULES_NAMES CORE_MODULES_DIRS doc)
set(CORE_MODULES_NAMES_UNSORTED ${CORE_MODULES_NAMES})
topological_sort(CORE_MODULES_NAMES CORE_ _DEPENDS)
sort_relative(CORE_MODULES_NAMES_UNSORTED CORE_MODULES_NAMES CORE_MODULES_DIRS)
foreach(subdir ${CORE_MODULES_DIRS})
  add_subdirectory(${CORE_SOURCE_DIR}/${subdir})
endforeach(subdir)

### ---[ Documentation
add_subdirectory(doc)

### ---[ Configure COREConfig.cmake
include(${CORE_SOURCE_DIR}/cmake/core_coreconfig.cmake)

### ---[ Package creation
include(${CORE_SOURCE_DIR}/cmake/core_all_in_one_installer.cmake)
include(${CORE_SOURCE_DIR}/cmake/core_cpack.cmake)

if(CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")
  CORE_MAKE_CPACK_INPUT()
  set(CPACK_PROJECT_CONFIG_FILE "${CORE_CPACK_CFG_FILE}")
  include(CPack)
endif(CPACK_GENERATOR)

### ---[ Make a pretty picture of the dependency graph
include(${CORE_SOURCE_DIR}/cmake/dep_graph.cmake)
MAKE_DEP_GRAPH()

### ---[ Finish up
CORE_WRITE_STATUS_REPORT()
CORE_RESET_MAPS()
