INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_SM200A sm200a)

FIND_PATH(
    SM200A_INCLUDE_DIRS
    NAMES sm200a/api.h
    HINTS $ENV{SM200A_DIR}/include
        ${PC_SM200A_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    SM200A_LIBRARIES
    NAMES gnuradio-sm200a
    HINTS $ENV{SM200A_DIR}/lib
        ${PC_SM200A_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SM200A DEFAULT_MSG SM200A_LIBRARIES SM200A_INCLUDE_DIRS)
MARK_AS_ADVANCED(SM200A_LIBRARIES SM200A_INCLUDE_DIRS)

