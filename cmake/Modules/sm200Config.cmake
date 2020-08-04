INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_SM200 sm200)

FIND_PATH(
    SM200_INCLUDE_DIRS
    NAMES sm200/api.h
    HINTS $ENV{SM200_DIR}/include
        ${PC_SM200_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    SM200_LIBRARIES
    NAMES gnuradio-sm200
    HINTS $ENV{SM200_DIR}/lib
        ${PC_SM200_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SM200 DEFAULT_MSG SM200_LIBRARIES SM200_INCLUDE_DIRS)
MARK_AS_ADVANCED(SM200_LIBRARIES SM200_INCLUDE_DIRS)

