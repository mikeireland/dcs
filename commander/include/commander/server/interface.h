#ifndef COMMANDER_SERVER_INTERFACE_H
#define COMMANDER_SERVER_INTERFACE_H

#include <commander/module.h>

namespace commander::server
{

    struct Interface
    {
        Interface(Module& module_) : module_(module_) {}

        virtual ~Interface() = default;

        virtual void run() = 0;

        Module& module_;

    };

} // namespace commander::server

#endif //COMMANDER_SERVER_INTERFACE_H