#include <commander/client/socket.h>
#include <iostream>

namespace co = commander;

int main()
{
    co::client::Socket socket("tcp://localhost:9999");

    auto result = socket.send<int>("add", 1, 2);
    std::cout << result << std::endl;
    return 0;
}