INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_WAVEGEN wavegen)

FIND_PATH(
    WAVEGEN_INCLUDE_DIRS
    NAMES wavegen/api.h
    HINTS $ENV{WAVEGEN_DIR}/include
        ${PC_WAVEGEN_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    WAVEGEN_LIBRARIES
    NAMES gnuradio-wavegen
    HINTS $ENV{WAVEGEN_DIR}/lib
        ${PC_WAVEGEN_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(WAVEGEN DEFAULT_MSG WAVEGEN_LIBRARIES WAVEGEN_INCLUDE_DIRS)
MARK_AS_ADVANCED(WAVEGEN_LIBRARIES WAVEGEN_INCLUDE_DIRS)

