project(GUIControl)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS
                      PROTO_HDRS
                      ../MovementControl/roboclaw_msgs.proto)

find_package( Qt5 COMPONENTS Widgets Concurrent REQUIRED )

# Qt auto moc and auto uic.
set( CMAKE_AUTOMOC ON )
set( CMAKE_AUTOUIC ON )
set( CMAKE_AUTORCC ON )

add_executable( ${PROJECT_NAME} )
target_sources( ${PROJECT_NAME} PRIVATE main.cpp movementwidget.cpp ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} PUBLIC -lpthread -lrt ${PROTOBUF_LIBRARIES} yaml-cpp )
target_link_libraries( ${PROJECT_NAME} PUBLIC -L/usr/local/lib -lSepiaComm -lzmq )
target_link_libraries( ${PROJECT_NAME} PUBLIC Qt5::Concurrent Qt5::Widgets )