project(MovementControl)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS
                      PROTO_HDRS
                      roboclaw_msgs.proto)

add_executable( ${PROJECT_NAME} )
target_sources( ${PROJECT_NAME} PRIVATE main.cpp roboclawinterface.cpp roboclawinterface.h ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} -L/usr/local/lib -lSepiaStream -lRoboClaw -lzmq ${PROTOBUF_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} -lpthread -lrt )
