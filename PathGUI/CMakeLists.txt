project(PathGUI)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS
                      PROTO_HDRS
                      ../PathFollower/path_msgs.proto)

find_package( Qt5 COMPONENTS Widgets REQUIRED )

# Qt auto moc.
set( CMAKE_AUTOMOC ON )

add_executable( ${PROJECT_NAME} )
target_sources( ${PROJECT_NAME} PRIVATE main.cpp pathcanvas.cpp pathcanvas.h pathgeometry.cpp pathgeometry.h pathwindow.cpp pathwindow.h ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( ${PROJECT_NAME} PUBLIC -lpthread -lrt ${PROTOBUF_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} PUBLIC -L/usr/local/lib -lSepiaComm -lzmq yaml-cpp )
target_link_libraries( ${PROJECT_NAME} PUBLIC Qt5::Widgets )
