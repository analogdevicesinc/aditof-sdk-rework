# Create the install directory first 
file(MAKE_DIRECTORY ${PROTOBUF_STATIC_LIB_DIRECTORY})

# Execute install commands for protobuf
execute_process(COMMAND cmake -Dprotobuf_BUILD_TESTS=OFF 
                            -Dprotobuf_MSVC_STATIC_RUNTIME=OFF 
                            -Dprotobuf_MODULE_COMPATIBLE:BOOL=TRUE
                            -DCMAKE_INSTALL_PREFIX=./local_path/protobuf 
                            -G${CMAKE_GENERATOR} ../src/cmake
				WORKING_DIRECTORY ${PROTOBUF_STATIC_LIB_DIRECTORY}
				)

if (WIN32)
    set(_PROTOBUF_CONFIG_DIRECTORY ${PROTOBUF_STATIC_LIB_DIRECTORY}/cmake)
else()
    set(_PROTOBUF_CONFIG_DIRECTORY ${PROTOBUF_STATIC_LIB_DIRECTORY}/lib/cmake/protobuf)
endif()

include(${_PROTOBUF_CONFIG_DIRECTORY}/protobuf-config.cmake)
include(${_PROTOBUF_CONFIG_DIRECTORY}/protobuf-module.cmake)
include(${_PROTOBUF_CONFIG_DIRECTORY}/protobuf-options.cmake)
include(${_PROTOBUF_CONFIG_DIRECTORY}/protobuf-targets.cmake) 

execute_process(COMMAND cmake --build . --target install --config Debug
				WORKING_DIRECTORY ${PROTOBUF_STATIC_LIB_DIRECTORY}
				)
execute_process(COMMAND cmake --build . --target install --config Release
				WORKING_DIRECTORY ${PROTOBUF_STATIC_LIB_DIRECTORY}
				)
