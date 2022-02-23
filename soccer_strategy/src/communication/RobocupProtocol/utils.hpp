#ifndef PROTOCOL_UTILS_HPP
#define PROTOCOL_UTILS_HPP

#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/util/json_util.h>
#include <chrono>
#include <iostream>
#include <string>

inline std::chrono::time_point<std::chrono::system_clock> convert_timestamp(
    const ::google::protobuf::Timestamp& timestamp) {
    return std::chrono::time_point<std::chrono::system_clock>(std::chrono::seconds(timestamp.seconds())
                                                              + std::chrono::nanoseconds(timestamp.nanos()));
}

template <typename T>
std::string team_colour(const T& team) {
    switch (team) {
        case T::BLUE: return "BLUE";
        case T::RED: return "RED";
        default: return "UNKNOWN";
    }
}

template <typename T>
std::string state(const T& state) {
    switch (state) {
        case T::PENALISED: return "PENALISED";
        case T::UNPENALISED: return "UNPENALISED";
        default: return "UNKNOWN";
    }
}

template <typename T>
inline void print_message(const T& msg) {
    ::google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace                = true;
    options.always_print_primitive_fields = true;

    std::string json_output;
    ::google::protobuf::util::MessageToJsonString(msg, &json_output, options);

    std::cout << "Parsed Message:" << std::endl;
    std::cout << json_output << std::endl;
}

template <typename T>
inline void parse_message(const std::string& string_msg) {
    // Parse the message using the official protobuf message format
    T msg;

    // Parse the received message, this will ignore any IDs that aren't defined in the official message
    msg.ParseFromString(string_msg);

    print_message<T>(msg);
}

#endif  // PROTOCOL_UTILS_HPP