#include <commander/client/socket.h>

namespace commander::client
{

    Socket::Socket(std::string socket) : ctx(1),
                                         sock(ctx, zmq::socket_type::req)
    {
        sock.connect(socket);
    }

    void Socket::send(std::string_view command_name, json args)
    {
        auto message = fmt::format("{} {}", command_name, args.dump());
        sock.send(zmq::message_t(message.c_str(), message.size()), zmq::send_flags::none);
    }

    json Socket::recv()
    {
        zmq::message_t msg;
        auto recv_res = sock.recv(msg, zmq::recv_flags::none);
        if (!recv_res)
        {
            throw std::runtime_error("[socket] error receiving message");
        }

        auto result = std::string(static_cast<char *>(msg.data()), msg.size());

        fmt::print("Received: {}\n", result);

        return json::parse(result);
    }

} // namespace commander::client
