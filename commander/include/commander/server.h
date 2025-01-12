#ifndef COMMANDER_SERVER_H
#define COMMANDER_SERVER_H

#include <commander/server/interactive.h>
#include <commander/module.h>

#include <memory>

namespace commander
{

    struct Server
    {

        Server(int argc, char** argv);

        void run();

        Module module_;
        std::shared_ptr<server::Interface> impl;


    };

} // namespace commander

#endif //COMMANDER_SERVER_H