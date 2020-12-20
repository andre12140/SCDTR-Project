#include "serverMultiple.h"

int main()
{
    io_context io;
    server s{io, 10000};
    io.run();
}