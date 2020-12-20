#include <iostream>
#include <boost/asio.hpp>
using namespace boost::asio;
using boost::system::error_code;
class session
{
    char rx_buf[1];

public:
    ip::tcp::socket sock;
    session(io_service &io) : sock{io} {}
    void start()
    {
        sock.async_read_some(buffer(rx_buf, 1),
                             [this](const error_code &ec, size_t sz) {
                                 if (!ec && rx_buf[0] != 'q')
                                     async_write(sock, buffer("Hello World\n"),
                                                 [this](const error_code &ec, size_t sz) {
                if (!ec)
                    start(); });
                             });
    }
};