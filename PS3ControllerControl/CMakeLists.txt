project(PS3ControllerControl)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS
                      PROTO_HDRS
                      robotics_msgs.proto
                      ../MovementControl/roboclaw_msgs.proto)

add_executable( ${PROJECT_NAME} )
target_sources( ${PROJECT_NAME} PRIVATE joystick.cpp joystick.h ps3controller.cpp ps3controller.h translator.cpp translator.h main.cpp ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} -lpthread -lrt ${PROTOBUF_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} -L/usr/local/lib -lSepiaStream -lzmq )
