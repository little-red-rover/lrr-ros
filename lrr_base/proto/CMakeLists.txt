set(Protobuf_PROTOC_EXECUTABLE /usr/bin/protoc)

add_library(proto)
target_include_directories(proto
    PUBLIC
    .
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Protobuf_INCLUDE_DIRS}
)
find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_SRC PROTO_HEADER
    messages.proto
)
target_link_libraries(proto ${Protobuf_LIBRARIES})
target_sources(proto
    PRIVATE
    ${PROTO_HEADER}
    ${PROTO_SRC}
)
