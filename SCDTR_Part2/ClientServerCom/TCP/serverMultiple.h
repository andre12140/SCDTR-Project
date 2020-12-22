
#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
using namespace boost::asio;
using namespace boost::system;
using ec = const boost::system::error_code &;

// Server - Arduino Hub Communication
boost::asio::io_context io_arduino;
boost::asio::serial_port sp{io_arduino};
boost::system::error_code ec_arduino;
boost::asio::streambuf read_buf; //read buffer

class session
{
    // Server-Client Communication
    ip::tcp::socket sock;
    char buf[1];
    char command[10]; // Command read from client
    int cmd_cntr = 0;

public:
    void check_valid_command()
    {
        if (command[0] == 'g')
        {
            if (command[2] == 'l')
            { // Get current measured illuminance at desk <i>.
            }
            else if (command[2] == 'd')
            { // Get current duty cycle at luminaire i
            }
            else if (command[2] == 'o')
            { // Get current occupancy state at desk <i>
            }
            else if (command[2] == 'O')
            { // Get lower bound on illuminance for Occupied state at desk <i>
            }
            else if (command[2] == 'U')
            { // Get lower bound on illuminance for Unoccupied state at desk <i>
            }
            else if (command[2] == 'L')
            { // Get current illuminance lower bound at desk <i>
            }
            else if (command[2] == 'x')
            { // Get current external illuminance at desk <i>
            }
            else if (command[2] == 'r')
            { // Get current illuminance control reference at desk <i>
            }
            else if (command[2] == 'c')
            { // Get current energy cost at desk <i>
            }
            else if (command[2] == 'p')
            {
                if (command[4] == 'T')
                { // Get instantaneous total power consumption in the system.
                }
                else
                { // Get instantaneous power consumption at desk <i>
                }
            }
            else if (command[2] == 't')
            { // Get elapsed time since last restart
            }
            else if (command[2] == 'e')
            {
                if (command[4] == 'T')
                { // Get total accumulated energy consumption since last system restart.
                }
                else
                { // Get accumulated energy consumption at desk <i> since the last system restart.
                }
            }
            else if (command[2] == 'v')
            {
                if (command[4] == 'T')
                { // Get total visibility error since last system restart.
                    std::cout << "command gvt" << std::endl;
                }
                else
                { // Get accumulated visibility error at desk <i> since the last system restart.
                }
            }
            else if (command[2] == 'f')
            {
                if (command[4] == 'T')
                { // Get total flicker error since last system restart.
                }
                else
                { // Get accumulated flicker error at desk <i> since the last system restart.
                }
            }
        }

        else if (command[0] == 'o') // Set current occupancy state at desk <i>
        {
        }
        else if (command[0] == 'O') //Set lower bound on illuminance for occupancy state at desk <i>
        {
        }
        else if (command[0] == 'U') // Set lower bound on illuminance for Unccupied state at desk <i>
        {
        }
        else if (command[0] == 'c') // Set current energy cost at desk <i>
        {
        }
        else if (command[0] == 'r') // Restart system
        {
        }
        else if (command[0] == 'b') // Get last minute buffer of variable <x> of desk <i>. <x> can be “l” or “d”.
        {
        }
        else if (command[0] == 's') //Start/Stop stream of realtime variable <x> of desk <i>. <x> can be “l” or “d”.
        {
        }
    }
    session(io_context &io) : sock(io) {}
    ip::tcp::socket &socket() { return sock; }

    static void write_handler(const error_code &ec_, size_t nbytes) // Called after writing a command
    {                                                               // waits for arduino response
                                                                    // [ADICIONAR] timeout se demorar a responder
        std::cout << "in async handler" << std::endl;
        // async_read_until(sp, read_buf, '\n', [this](ec, std::size_t sz) {
        //     std::cout << "FROM ARDUINO: " << &read_buf << std::endl; // Called after writing a command; waits for arduino response
        //      enviar para cliente (formatar o trabalho)!!
        // });  [ADICIONAR] timeout se demorar a responder
    }

    void serve_client_request()
    {

        sock.async_read_some(buffer(buf, 1),
                             [this](ec err, size_t sz) {
                                 std::cout << "Received \"" << buf[0] << "\" from: ";
                                 std::cout << sock.remote_endpoint() << std::endl;
                                 if (buf[0] != 'q' && !err)
                                 {

                                     //async_write(sock, buffer("antes da ecursividade"), [this](ec, std::size_t sz) {});
                                     command[cmd_cntr++] = buf[0]; // update command with current byet

                                     //std::cout << buf[0] << std::endl; // debug
                                     if (buf[0] == '\n')
                                     { //  End of command '\n'
                                         std::cout << "FROM CLIENT: " << command << std::endl;
                                         //check_valid_command();
                                         async_write(sp, buffer(command), write_handler); //Writes to Arduino HUB (Serial Port)

                                         //async_write(sock, buffer(command), [this](ec, std::size_t sz) {}); //Writes to client (TCP Socket)
                                         memset(command, 0, 10); // CLear buffer
                                         cmd_cntr = 0;           // Sets counter to zero after command read
                                     }

                                     serve_client_request();
                                 }
                                 else
                                     delete this;
                             } //end async_read lambda arg
        );                     //end async_read call
    }                          //end start()
};

class server
{
    io_context &ctx;
    ip::tcp::acceptor acc;
    void start_accept()
    {
        session *sess = new session{ctx};
        acc.async_accept(sess->socket(),
                         [this, sess](ec err) {
                             if (!err)
                                 sess->serve_client_request();

                             else
                                 delete sess;
                             start_accept();
                         } //end async_accept lambda arg
        );                 //end async_accept cal
    }                      //end start_accept();
public:
    server(io_service &io, unsigned short port) : ctx{io},
                                                  acc{io, ip::tcp::endpoint{ip::tcp::v4(), port}}
    {
        std::cout << "Receiving at: " << acc.local_endpoint() << std::endl;
        start_accept();
    }
};