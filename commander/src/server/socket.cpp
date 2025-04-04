#include <commander/server/socket.h>

#include <fmt/core.h>
#include <iostream>

namespace commander::server
{

    Socket::Socket(Module& module_, std::string socket):
        Interface(module_),
        ctx(1),
        // sock will listen for incoming messages
        sock(ctx, zmq::socket_type::rep)
    {
        fmt::print("[socket] binding to {}\n", socket);
        // sock will listen from socket
        sock.bind(socket);
    }


    void Socket::run()
    {

        zmq::message_t msg;
        while (true)
        {
            auto recv_res = sock.recv(msg, zmq::recv_flags::none);
            if (!recv_res)
            {
                fmt::print("[socket] error receiving message\n");
                continue;
            }

            auto command = std::string(static_cast<char*>(msg.data()), msg.size());

            fmt::print("Received command: {}\n", command);

            auto pos = command.find(' ');
            std::string name;
            json args;

            try {
                if (pos == std::string::npos)
                    name = command;
                    if (command == "exit")
                        break;
                else
                {
                    name = command.substr(0, pos);
                    command = fmt::format("[{}]", command.substr(pos + 1));
                    args = json::parse(command);
                }

                auto res = module_.execute(name, args).dump();

                fmt::print("Sending response: {}\n", res);

                sock.send(zmq::message_t(res.c_str(), res.size()), zmq::send_flags::none);
            } catch (const std::exception& e) {
                std::string error = fmt::format("Error: {}", e.what());
                fmt::print("{}\n", error);
                sock.send(zmq::message_t(error.c_str(), error.size()), zmq::send_flags::none);
            }
        }
    }
}
