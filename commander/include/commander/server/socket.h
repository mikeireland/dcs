#ifndef COMMANDER_SERVER_SOCKET_H
#define COMMANDER_SERVER_SOCKET_H

#include <commander/server/interface.h>

#include <zmq.hpp>

namespace commander::server
{

    struct Socket : Interface
    {
        Socket(Module& module_, std::string socket);

        ~Socket() override = default;

        void run() override;

        zmq::context_t ctx;
        zmq::socket_t sock;
    };

} // namespace commander::server

#endif //COMMANDER_SERVER_SOCKET_H