#ifndef COMMANDER_SERVER_SINGLE_COMMAND_H
#define COMMANDER_SERVER_SINGLE_COMMAND_H

#include <commander/server/interface.h>

#include <string>

namespace commander::server
{

    struct SingleCommand : Interface
    {
        SingleCommand(Module& module_, std::string command);

        ~SingleCommand() override = default;

        void run() override;

        std::string command;
    };

} // namespace commander::server

#endif //COMMANDER_SERVER_SINGLE_COMMAND_H