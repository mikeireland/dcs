cmake_minimum_required(VERSION 3.18)

project(commander VERSION 1.0.0 LANGUAGES CXX)

# Always generate compile_commands.json. Used by clangd and other tools.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(commander
    src/registry.cpp
    src/server.cpp
    src/module.cpp
    src/server/interactive.cpp
    src/server/single_command.cpp
    src/server/socket.cpp
    src/client/socket.cpp
)

target_sources(commander
PUBLIC FILE_SET HEADERS BASE_DIRS include FILES
    include/commander/client/socket.h
    include/commander/commander.h
    include/commander/function_parser.h
    include/commander/module.h
    include/commander/registry.h
    include/commander/server.h
    include/commander/server/interactive.h
    include/commander/server/interface.h
    include/commander/server/single_command.h
    include/commander/server/socket.h
)

target_compile_features(commander PUBLIC cxx_std_17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(fmt REQUIRED)

target_link_libraries(commander PUBLIC
    Boost::boost
    Boost::program_options
    fmt::fmt
    zmq
)

install(TARGETS commander
    FILE_SET HEADERS
)


add_subdirectory(examples)
