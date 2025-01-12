#ifndef COMMANDER_SERVER_INTERACTIVE_H
#define COMMANDER_SERVER_INTERACTIVE_H

#include <commander/server/interface.h>

namespace commander::server
{

    struct Interactive : Interface
    {
        Interactive(Module& module_);

        ~Interactive() override = default;

        void run() override;

    };

} // namespace commander::server

#endif //COMMANDER_SERVER_INTERACTIVE_H