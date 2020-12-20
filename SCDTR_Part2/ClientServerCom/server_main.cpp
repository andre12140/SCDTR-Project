#include "server.h"

int main()
{
    io_context io;
    ip::tcp::acceptor acc{io, ip::tcp::endpoint{ip::tcp::v4(), 10000}};
    for (;;)
    {
        session client{io};
        std::cout << "Waiting for new client\n";
        acc.async_accept(client.sock,
                         [&client](const error_code &ec) {
                                                          client.start();
                         });
        io.run();
        std::cout << "Ending session\n";
        io.restart();
    }
    // session goes out of scope - old socket released
}