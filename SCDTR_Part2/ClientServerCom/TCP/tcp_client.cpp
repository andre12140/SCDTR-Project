//SYNC_TCP_CLIENT.CPP
#include <iostream>
#include <boost/asio.hpp>
using namespace boost::asio;
int main()
{
    io_context io;
    boost::system::error_code err;
    char buf[128];
    ip::tcp::resolver res{io}; //find endpoints from address
    ip::tcp::socket client_sock{io};
    connect(client_sock, res.resolve("127.0.0.1", "10000"), err);
    if (err)
    {
        std::cout << "Cannot connect" << std::endl;
        return 1;
    }
    do
    {
        std::cin.getline(buf, 128);
        if (strlen(buf) == 0)
            continue; //empty line
        write(client_sock, buffer(buf, strlen(buf)), err);
        size_t n = client_sock.read_some(buffer(buf, 128), err);
        std::cout.write(buf, n);
    } while (err.value() == 0 && buf[0] != 'q'); //kills connection
}