#include <commander/server/interactive.h>

#include <fmt/core.h>
#include <iostream>

namespace commander::server
{

    Interactive::Interactive(Module& module_):
        Interface(module_)
    {}

    void Interactive::run()
    {
        std::cout << "Interactive\n> ";

        std::string command;

        while(std::getline(std::cin, command))
        {
            if (command == "exit")
                break;

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
            fmt::print("{}\n> ", res.dump());
        }
    }


}