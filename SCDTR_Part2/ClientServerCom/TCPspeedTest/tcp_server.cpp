//SYNC_TCP_SERVER.CPP
#include <iostream>
#include <boost/asio.hpp>
using namespace boost::asio;
int main()
{
    io_context io;
    boost::system::error_code err;
    char rx_buf[5] = {0};
    char tx_buf[13] = {"Hello World\n"};
    ip::tcp::acceptor acc{io, ip::tcp::endpoint{ip::tcp::v4(), 10000}};
    std::cout << "Listening at : " << acc.local_endpoint() << std::endl;
    for (;;)
    {
        ip::tcp::socket sock{io}; //create listening socket
        acc.accept(sock);         //wait client to connect
        do
        { //got a client
            sock.read_some(buffer(rx_buf, 5), err);
            if (rx_buf[0] != 'q')
                write(sock, buffer(rx_buf, 5), err);
            else
            {
                write(sock, buffer(rx_buf, 5), err);
            }
        } while (err.value() == 0 && rx_buf[0] != 'q');
        //kills connection
    }
}