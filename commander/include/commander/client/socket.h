#ifndef COMMANDER_CLIENT_SOCKET_H
#define COMMANDER_CLIENT_SOCKET_H

#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <fmt/core.h>

#include <future>

namespace commander::client
{
    using nlohmann::json;

    struct Socket
    {
        Socket(std::string socket);

        ~Socket() = default;

        template <typename Res, typename... Ts>
        Res send(std::string_view command_name, Ts... ts) {
            json args = json::array({json(ts)...});

            send(command_name, args);

            auto result = recv();

            return result.get<Res>();

        }

        template <typename Res, typename... Ts>
        std::future<Res> send_async(std::string_view command_name, Ts... ts) {
            return std::async(std::launch::async, [this, command_name, ts...]() {
                return send<Res>(command_name, ts...);
            });

        }

        void send(std::string_view command_name, json args);

        json recv();

        zmq::context_t ctx;
        zmq::socket_t sock;
    };

} // namespace commander::client

#endif // COMMANDER_CLIENT_SOCKET_H