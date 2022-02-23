cmake_minimum_required(VERSION 3.0)
project(RobocupProtocol)

# We need protobuf to generate the messages
find_package(Protobuf REQUIRED)

function(COMPILE_PROTO
         proto_file
         output_dir
         return_var)
  # Get the file without the extension
  get_filename_component(file_we ${proto_file} NAME_WE)

  # Run the protocol buffer compiler on the builtin protocol buffers
  add_custom_command(OUTPUT "${output_dir}/${file_we}.pb.cc"
                            "${output_dir}/${file_we}.pb.h"
                     COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                             ARGS
                             --cpp_out=${output_dir}
                             -I${CMAKE_CURRENT_SOURCE_DIR}
                             ${proto_file}
                     DEPENDS ${proto_file}
                     COMMENT "Compiling protocol buffer ${proto_file}")

  set_source_files_properties(
    "${output_dir}/${file_we}.pb"
    "${output_dir}/${file_we}.proto"
    "${output_dir}/${file_we}.pb.cc"
    "${output_dir}/${file_we}.pb.h"
    "${output_dir}/${file_we}.cpp"
    "${output_dir}/${file_we}.py.cpp"
    "${output_dir}/${file_we}.h"
    PROPERTIES
    GENERATED
    TRUE
    # Prevent Effective C++ and unused parameter error checks being performed on
    # generated files.
    COMPILE_FLAGS "-Wno-unused-parameter -Wno-error=unused-parameter -Wno-error"
    )

  set(${return_var} "${output_dir}/${file_we}.pb.cc"
                    "${output_dir}/${file_we}.pb.h" PARENT_SCOPE)
endfunction(COMPILE_PROTO)

# Build our protobuf classes
compile_proto("${CMAKE_CURRENT_SOURCE_DIR}/robocup.proto"
              "${CMAKE_CURRENT_BINARY_DIR}" base_message)
compile_proto("${CMAKE_CURRENT_SOURCE_DIR}/robocup_extension.proto"
              "${CMAKE_CURRENT_BINARY_DIR}" extended_message)

message(STATUS ${base_message})
message(STATUS ${extended_message})

add_executable(encode_base encode_base.cpp ${base_message})
target_include_directories(encode_base PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(encode_base protobuf::libprotobuf)

add_executable(decode_base decode_base.cpp ${base_message})
target_include_directories(decode_base PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(decode_base protobuf::libprotobuf)

add_executable(encode_extended encode_extended.cpp ${extended_message})
target_include_directories(encode_extended PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(encode_extended protobuf::libprotobuf)

add_executable(decode_extended decode_extended.cpp ${extended_message})
target_include_directories(decode_extended PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(decode_extended protobuf::libprotobuf)
