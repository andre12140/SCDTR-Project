#
#include <iostream>
#include <string>
#include <sstream>
#include <boost/asio.hpp>
#include <thread>
#include "message.h"
#include <ctime>

using namespace boost::asio;
using namespace boost::system;
using ec = const boost::system::error_code &;

#define MSG_SIZE 64
#define MAX_CLIENTS 8

// Server - Arduino Hub Communication
io_context io;
serial_port sp{io};
streambuf read_buf; //read buffer

// DEBUG -------------
std::clock_t start;
double duration;

size_t nBufferSize;
char buf_cpy[MSG_SIZE];

class session;
struct client_info
{
    session *sess_obj;
    int id;
};

client_info clients[8]; // Array with list of current clients connected to server

class session
{
    // Server-Client Communication

    char buf[1];
    char command[MSG_SIZE];     // Command read from client
    char command_cpy[MSG_SIZE]; // aux variable to save command
    int cmd_cntr = 0;

    std::stringstream ssOut;
    std::__cxx11::string received_msg;

public:
    ip::tcp::socket sock{io};
    session(io_context &io) : sock(io) {}

    int client_id;
    ip::tcp::socket &socket() { return sock; }

    bool check_valid_cmd(char *cmd_cv, char *msg, uint8_t cl_id)
    {
        if (cmd_cv[1] != ' ')
        {
            return false;
        }

        if (cmd_cv[0] == 'g')
        {
            if (cmd_cv[2] == 'l')
            { // Get current measured illuminance at desk <i>.
                msg[0] = (gl | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'd')
            { // Get current duty cycle at luminaire i
                msg[0] = (gd | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'o')
            { // Get current occupancy state at desk <i>
                msg[0] = (go | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'O')
            { // Get lower bound on illuminance for Occupied state at desk <i>
                msg[0] = (gO | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'U')
            { // Get lower bound on illuminance for Unoccupied state at desk <i>
                msg[0] = (gU | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'L')
            { // Get current illuminance lower bound at desk <i>
                msg[0] = (gL | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'x')
            { // Get current external illuminance at desk <i>
                msg[0] = (gx | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'r')
            { // Get current illuminance control reference at desk <i>
                msg[0] = (gr | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'c')
            { // Get current energy cost at desk <i>
                msg[0] = (gc | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'p')
            {
                if (cmd_cv[4] == 'T')

                { // Get instantaneous total power consumption in the system.
                    msg[0] = (gpT | 0x80) & 0xBF;
                }

                else
                { // Get instantaneous power consumption at desk <i>
                    msg[0] = (gp | 0x80) & 0xBF;
                    char subs_string[10] = {0}; // Conversion of string ID to int
                    memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                    int x = atoi(subs_string);
                    memcpy(&msg[1], &x, 1);
                }
            }

            else if (cmd_cv[2] == 't')

            { // Get elapsed time since last restart
                msg[0] = (gt | 0x80) & 0xBF;
                char subs_string[10] = {0}; // Conversion of string ID to int
                memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                int x = atoi(subs_string);
                memcpy(&msg[1], &x, 1);
            }

            else if (cmd_cv[2] == 'e')
            {

                if (cmd_cv[4] == 'T')
                { // Get total accumulated energy consumption since last system restart.
                    msg[0] = (geT | 0x80) & 0xBF;
                }

                else
                { // Get accumulated energy consumption at desk <i> since the last system restart.
                    msg[0] = (ge | 0x80) & 0xBF;
                    char subs_string[10] = {0}; // Conversion of string ID to int
                    memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                    int x = atoi(subs_string);
                    memcpy(&msg[1], &x, 1);
                }
            }

            else if (cmd_cv[2] == 'v')
            {

                if (cmd_cv[4] == 'T')

                { // Get total visibility error since last system restart.
                    msg[0] = (gvT | 0x80) & 0xBF;
                }

                else
                { // Get accumulated visibility error at desk <i> since the last system restart.
                    msg[0] = (gv | 0x80) & 0xBF;
                    char subs_string[10] = {0}; // Conversion of string ID to int
                    memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                    int x = atoi(subs_string);
                    memcpy(&msg[1], &x, 1);
                }
            }

            else if (cmd_cv[2] == 'f')
            {
                if (cmd_cv[4] == 'T')
                { // Get total flicker error since last system restart.
                    msg[0] = (gfT | 0x80) & 0xBF;
                }

                else
                { // Get accumulated flicker error at desk <i> since the last system restart.
                    msg[0] = (gf | 0x80) & 0xBF;
                    char subs_string[10] = {0}; // Conversion of string ID to int
                    memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);
                    int x = atoi(subs_string);
                    memcpy(&msg[1], &x, 1);
                }
            }
        }

        else if (cmd_cv[0] == 'o') // Set current occupancy state at desk <i>
        {
            msg[0] = (o | 0x80) & 0xBF;
            char subs_string[10] = {0}; // Conversion of string ID to int
            memcpy(subs_string, &cmd_cv[2], strlen(cmd_cv) - 2 - 1);

            char *tok;
            tok = strtok(subs_string, " ");
            int x = atoi(tok);
            memcpy(&msg[1], &x, 1); // <i>
            tok = strtok(NULL, " ");
            bool val = atoi(tok);
            memcpy(&msg[2], &val, 1); // <val>
        }

        else if (cmd_cv[0] == 'O') //Set lower bound on illuminance for occupancy state at desk <i>
        {
            msg[0] = (O | 0x80) & 0xBF;
            char subs_string[10] = {0};                              // Conversion of string ID to int
            memcpy(subs_string, &cmd_cv[2], strlen(cmd_cv) - 2 - 1); // val

            char *tok;
            tok = strtok(subs_string, " ");
            int x = atoi(tok);
            memcpy(&msg[1], &x, 1); // <i>
            tok = strtok(NULL, " ");
            float y = atof(tok);
            y += 0.01;
            memcpy(&msg[2], &y, sizeof(float)); // <val>
        }

        else if (cmd_cv[0] == 'U') // Set lower bound on illuminance for Unccupied state at desk <i>
        {
            msg[0] = (U | 0x80) & 0xBF;
            char subs_string[10] = {0}; // Conversion of string ID to int
            memcpy(subs_string, &cmd_cv[2], strlen(cmd_cv) - 2 - 1);

            char *tok;
            tok = strtok(subs_string, " ");
            int x = atoi(tok);
            memcpy(&msg[1], &x, 1); // <i>
            tok = strtok(NULL, " ");
            float y = atof(tok);
            y += 0.01;
            memcpy(&msg[2], &y, sizeof(float)); // <val>
        }

        else if (cmd_cv[0] == 'c') // Set current energy cost at desk <i>
        {
            msg[0] = (c | 0x80) & 0xBF;
            char subs_string[10] = {0}; // Conversion of string ID to int
            memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);

            char *tok;
            tok = strtok(subs_string, " ");
            int x = atoi(tok);
            memcpy(&msg[1], &x, 1); // <i>

            tok = strtok(subs_string, " ");
            float y = atof(tok);
            memcpy(&msg[2], &y, sizeof(float)); // <val>
        }

        else if (cmd_cv[0] == 'r') // Restart system
        {
            msg[0] = (r | 0x80) & 0xBF;
        }

        else if (cmd_cv[0] == 'b') // Get last minute buffer of variable <x> of desk <i>. <x> can be ???l??? or ???d???.
        {
            msg[0] = (b | 0x80) & 0xBF;
            char subs_string[10] = {0}; // Conversion of string ID to int
            memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);

            char *tok;
            tok = strtok(subs_string, " ");
            memcpy(&msg[1], &tok, 1); // "l" or "d"
            if (msg[1] != 'l' || msg[1] != 'd')
            {
                return false;
            }
            tok = strtok(subs_string, " ");
            int y = atoi(tok);
            memcpy(&msg[2], &y, 1); // <i>
        }

        else if (cmd_cv[0] == 's') //Start/Stop stream of realtime variable <x> of desk <i>. <x> can be ???l??? or ???d???.
        {
            msg[0] = (ss | 0x80) & 0xBF;
            char subs_string[10] = {0}; // Conversion of string ID to int
            memcpy(subs_string, &cmd_cv[4], strlen(cmd_cv) - 4 - 1);

            char *tok;
            tok = strtok(subs_string, " ");
            memcpy(&msg[1], &tok, 1); // "l" or "d"
            if (msg[1] != 'l' || msg[1] != 'd')
            {
                return false;
            }
            tok = strtok(subs_string, " ");
            int y = atoi(tok);
            memcpy(&msg[2], &y, 1); // <i>
        }

        else
        {
            return false;
        }
        msg[1] = (msg[1] & 0x1F) | (cl_id << 5);
        return true;
    }

    static void write_handler(ec &ec_, size_t nbytes) // Called after writing a command
    {                                                 // waits for arduino response
                                                      // [ADICIONAR] timeout se demorar a responder
        //std::cout << "WA Command successfully sent to Arduino" << std::endl;
    }

    void serve_client_request()
    {

        // Reads until '/n' = one command

        sock.async_read_some(buffer(buf, 1),
                             [this](ec err, size_t sz) {
                                 if (buf[0] != 'q' && !err)
                                 {

                                     command[cmd_cntr++] = buf[0]; // update command with current byte

                                     //std::cout << buf[0] << std::endl; // debug
                                     if (buf[0] == '\n')
                                     { //  End of command '\n'
                                         std::cout << "From client " << client_id << ": " << command;
                                         //std::cout << "AR Depois de ler o comando, in avail:" << read_buf.in_avail() << std::endl;
                                         //check_valid_command();

                                         memcpy(command_cpy, command, cmd_cntr);
                                         char msg[8] = {0};

                                         check_valid_cmd(command_cpy, msg, (uint8_t)client_id);
                                         start = std::clock();                        // get current time
                                         async_write(sp, buffer(msg), write_handler); //Writes to Arduino HUB (Serial Port)
                                         memset(command, 0, MSG_SIZE);                // CLear buffer
                                         cmd_cntr = 0;                                // Sets counter to zero after command read
                                     }

                                     serve_client_request();
                                 }
                                 else
                                 {
                                     clients[client_id].sess_obj = NULL;
                                     // remover o proprio ID da lista de clientes do Server
                                     std::cout
                                         << "User " << client_id << " disconnected..." << std::endl;
                                     delete this;
                                 }
                             } //end async_read lambda arg
        );                     //end async_read call
    }                          //end start()
};

class server
{

    io_context &ctx;
    ip::tcp::acceptor acc;
    void read_until_handler(ec err, size_t sz)
    {
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

        std::cout << "Operation took " << duration << "seconds" << std::endl;
        if (read_buf.in_avail() > 0) // If there's bytes to send in buffer
        {
            //std::cout << "RU Depois de ler do arduino, in avail:" << read_buf.in_avail() << std::endl;
            memset(buf_cpy, 0, MSG_SIZE);
            read_buf.sgetn(buf_cpy, sz); // Copy from read_buf (received msg from arduino)

            if (buf_cpy[0] == 'C') // MESSAGE TO CLIENT
            {
                int cl_id = atoi(&buf_cpy[1]);

                clients[cl_id].sess_obj->sock.wait(boost::asio::ip::tcp::socket::wait_write);
                async_write(clients[cl_id].sess_obj->sock, buffer(buf_cpy), [this](ec err, std::size_t sz) { //Writes to Client (TCP socket)
                    //std::cout << "RU Depois de escrever p cliente" << read_buf.in_avail() << std::endl; //read_buf.consume(read_buf.size());                   // Clears buffer
                    //std::cout << "RU Return successfully sent to Client" << std::endl;

                    if (err)
                    {
                        std::cout << err << std::endl;
                        exit(0);
                        std::cout << "SZ == 0" << std::endl;
                    }
                });
            }
            else if (buf_cpy[0] == 'D') // Debug
            {
                std::cout << "[DEBUG FROM ARDUINO] " << buf_cpy;
            }
        }
        async_read_until(sp, read_buf, '\n', [this](ec err, size_t sz) {
            read_until_handler(err, sz); // Reads from Arduino HUB (Serial Port)
        });
    }

    void serve_arduino_read()
    {
        std::cout << "New session..." << std::endl;
        async_read_until(sp, read_buf, '\n', [this](ec err, size_t sz) {
            read_until_handler(err, sz);
        }); // Reads from Arduino HUB (Serial Port)
    }

    int find_free_slot()
    { //Finds a client free slot
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            if (clients[i].sess_obj == NULL)
            {

                return i; // return ID (= index)
            }
        }
        return -1; // No free slots, can't serve more  than 8 clients
    }

    void start_accept()
    {

        session *sess = new session{ctx};
        acc.async_accept(sess->socket(),
                         [this, sess](ec err) {
                             if (!err)
                             {
                                 int cl_id = find_free_slot();
                                 if (cl_id == -1)
                                 {
                                     std::cout << "D Can't accept more clients" << std::endl; // Server already serving 8 clients
                                     delete sess;                                             // Can't accept more connections
                                 }
                                 else // Accepts client
                                 {
                                     std::cout << "D New client!" << std::endl;
                                     clients[cl_id] = {sess, cl_id};
                                     sess->client_id = cl_id;

                                     sess->serve_client_request();
                                 }
                             }
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

        serve_arduino_read();
        std::cout << "Receiving at: " << acc.local_endpoint() << std::endl;
        start_accept();
    }
};
