# Create build folder for glog
file(MAKE_DIRECTORY ${GLOG_STATIC_LIB_DIRECTORY})

set(BUILD_TESTING FALSE CACHE BOOL "skip unit test projects" FORCE)
set(WITH_GFLAGS FALSE CACHE BOOL "build without gflags")
add_subdirectory(src)
