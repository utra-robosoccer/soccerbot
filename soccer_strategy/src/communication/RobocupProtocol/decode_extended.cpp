#include <google/protobuf/timestamp.pb.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "robocup_extension.pb.h"
#include "utils.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <msg.pb>" << std::endl;
        return 0;
    }

    std::string msg_file(argv[1]);

    // Open the extended message and parse it
    std::ifstream ifs(msg_file, std::ifstream::binary);

    if (ifs.is_open()) {
        // Decode a message and print it
        std::stringstream stream;
        stream << ifs.rdbuf();
        parse_message<robocup::humanoid::Message>(stream.str());
    }

    return 0;
}
