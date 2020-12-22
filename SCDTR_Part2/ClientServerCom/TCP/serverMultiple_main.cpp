#include "serverMultiple.h"

int main()
{
    // Serial communication setup
    sp.open("/dev/ttyUSB0", ec_arduino); //connect to port
    if (ec_arduino)
        std::cout << "Could not open serial port \n";
    sp.set_option(serial_port_base::baud_rate{1000000}, ec_arduino);

    // TCP communication setup
    io_context io;
    server s{io, 10000};

    io.run();
    io_arduino.run();
}