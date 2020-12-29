

#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;

class client
{
public:
    ip::tcp::resolver res; // to find out the server coordinates
    ip::tcp::socket sock;  // socket object

    streambuf input_buffer; //buffer to store the incoming server messages

    posix::stream_descriptor input; // console object
    streambuf console_buffer;       // buffer to store the console input
    char send_buffer[1024];         // buffet to store the data to send to server

    void start_connect_server(char *host, char *port)
    {
        res.async_resolve(ip::tcp::resolver::query(host, port),
                          [this](const boost::system::error_code &err, ip::tcp::resolver::results_type results) {
                              async_connect(sock, results,
                                            [this](const boost::system::error_code &err, const ip::tcp::endpoint &ep) {
                                                std::cout << "Connected to " << ep << std::endl;
                                                start_read_server();
                                            });
                          });
    }

    void start_read_server()
    {
        std::cout << "Waiting user input..." << std::endl;
        async_read_until(sock, input_buffer, '\n',
                         [this](const boost::system::error_code &err, size_t sz) {
                             if (!err)
                             {
                                 std::string line;
                                 std::istream is{&input_buffer};
                                 std::getline(is, line);
                                 if (!line.empty())
                                 {
                                     std::cout << "[RECEIVED FROM SERVER]" << line << "\n";
                                 }
                                 start_read_server();
                             }
                             else
                             {
                                 std::cout << "Error on receive: " << err.message() << "\n";
                             }
                         });
    }

    void start_read_console()
    {
        async_read_until(input, console_buffer, '\n',
                         [this](const boost::system::error_code &err, size_t sz) {
                             if (!err)
                             {
                                 std::string line, terminated_line;
                                 std::istream is(&console_buffer);
                                 std::getline(is, line);

                                 if (!line.empty())
                                 { // Empty messages are ignored.
                                     terminated_line = line + std::string("\n");
                                     std::size_t n = terminated_line.size();
                                     terminated_line.copy(send_buffer, n);
                                     async_write(sock, buffer(send_buffer, n), [this](const boost::system::error_code &err, size_t sz) { std::cout << "Sent " << sz << " bytes" << std::endl; });
                                 }
                                 start_read_console();
                             }
                             else
                             {
                                 std::cout << "Error on read console handler: " << err.message() << "\n";
                             }
                         });
    }

    client(io_context &io) : res{io}, sock{io}, input{io, ::dup(STDIN_FILENO)}, console_buffer{100000} {}

    void start(char *host, char *port)
    {
        start_connect_server(host, port);
        start_read_console();
    }
};
