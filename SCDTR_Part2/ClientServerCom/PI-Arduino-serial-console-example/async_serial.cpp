#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
using namespace boost::system;
using namespace boost::asio;
boost::asio::io_context io;
boost::asio::serial_port sp{io};
boost::asio::steady_timer tim{io};
boost::asio::streambuf read_buf; //read buffer
int counter = 0;
//forward declaration of write_handler to timer_handler
void write_handler(const error_code &ec, size_t nbytes);
void timer_handler(const error_code &ec)
{
    //timer expired – launch new write operation
    std::ostringstream os;
    os << "Counter = " << ++counter << std::endl;
    async_write(sp, buffer(os.str()), write_handler);
}

void write_handler(const error_code &ec, size_t nbytes)
{
    tim.expires_after(boost::asio::chrono::seconds{2});
    tim.async_wait(timer_handler);
}
void read_handler(const error_code &ec, size_t nbytes)
{
    std::cout << &read_buf;
    async_read_until(sp, read_buf, '\n', read_handler);
}
int main()
{
    boost::system::error_code ec;
    sp.open("/dev/ttyUSB0", ec); //connect to port
    if (ec)
        std::cout << "Could not open serial port \n";
    sp.set_option(serial_port_base::baud_rate{1000000}, ec);
    //program timer for write operations
    tim.expires_after(boost::asio::chrono::seconds{2});
    tim.async_wait(timer_handler);
    //program chain of read operations
    async_read_until(sp, read_buf, '\n', read_handler);
    io.run(); //get things rolling
}
