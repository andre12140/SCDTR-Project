#include "client.h"

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: client <host> <port>\n";
        return 1;
    }
    io_context io;
    client cli{io};
    cli.start(argv[1], argv[2]);
    io.run();
    return 0;
}