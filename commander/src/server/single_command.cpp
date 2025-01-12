#include <commander/server/single_command.h>

#include <fmt/core.h>
#include <iostream>

namespace commander::server
{

    SingleCommand::SingleCommand(Module& module_, std::string command):
        Interface(module_),
        command(command)
    {}

    void SingleCommand::run()
    {
        auto pos = command.find(' ');
        std::string name;
        json args;

        if (pos == std::string::npos)
            name = command;
        else
        {
            name = command.substr(0, pos);
            args = json::parse(command.substr(pos + 1));
        }

        auto res = module_.execute(name, args);
        fmt::print("{}\n", res.dump());
    }

}// namespace commander::server
