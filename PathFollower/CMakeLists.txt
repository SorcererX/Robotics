project(PathFollower)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS
                      PROTO_HDRS
                      path_msgs.proto
                      ../MovementControl/roboclaw_msgs.proto)

add_executable( ${PROJECT_NAME} )
target_sources( ${PROJECT_NAME} PRIVATE main.cpp pathfollower.cpp pathfollower.h ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} -lpthread -lrt ${PROTOBUF_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} -L/usr/local/lib -lSepiaStream -lSepiaComm -lzmq yaml-cpp )
