project(ImuReader)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS
                      PROTO_HDRS
                      imu_msgs.proto)

# Yost Labs' C API for 3-Space v3 sensors. It has no releases yet, so a known commit is used.
# Its own CMakeLists.txt also builds a Windows example, so only the sources are taken from it.
# The patch fixes taking the address of va_list parameters, which does not compile on 64-bit Linux. It is
# only applied when it is not applied already, since the patch step can run again on a later configure.
set(TSS_PATCH ${CMAKE_CURRENT_SOURCE_DIR}/tss_c_api_va_list.patch)
include(FetchContent)
FetchContent_Declare(tss_c_api
                     GIT_REPOSITORY https://github.com/YostLabs/3Space-C_API.git
                     GIT_TAG 8230bf5aa6f2240cb9ba08e18f817d70d228580f
                     PATCH_COMMAND sh -c "git apply --reverse --check ${TSS_PATCH} 2>/dev/null || git apply ${TSS_PATCH}"
                     SOURCE_SUBDIR no-cmake)
FetchContent_MakeAvailable(tss_c_api)

file(GLOB_RECURSE TSS_API_SRCS ${tss_c_api_SOURCE_DIR}/api/*.c)
add_library(tss_api STATIC
            ${TSS_API_SRCS}
            ${tss_c_api_SOURCE_DIR}/communication/serial/serial_com_class.c
            ${tss_c_api_SOURCE_DIR}/communication/serial/linux_serial.c)
target_include_directories(tss_api PUBLIC ${tss_c_api_SOURCE_DIR}/include ${tss_c_api_SOURCE_DIR}/communication/include)
set_target_properties(tss_api PROPERTIES C_STANDARD 11 POSITION_INDEPENDENT_CODE ON)

add_executable( ${PROJECT_NAME} )
target_sources( ${PROJECT_NAME} PRIVATE main.cpp imureader.cpp imureader.h ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} tss_api -lpthread -lrt ${PROTOBUF_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} -L/usr/local/lib -lSepiaStream -lSepiaComm -lzmq yaml-cpp )
