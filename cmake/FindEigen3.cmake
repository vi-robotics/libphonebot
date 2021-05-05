if(NOT ANDROID)
include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(eigen3 REQUIRED eigen3)
else(PKG_CONFIG_FOUND)
    message(FATAL_ERROR "Could not find pkg-config to search for Eigen3.")
endif(PKG_CONFIG_FOUND)
else()
set(eigen3_INCLUDE_DIRS /usr/local/include/eigen3)
if(NOT EXISTS ${eigen3_INCLUDE_DIRS}/signature_of_eigen3_matrix_library)
message(FATAL_ERROR "eigen3 could not be found, make sure you installed it in your
android toolchain")
endif()
endif()
