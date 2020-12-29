#include "serverMultiple.h"

int main()
{
    // Serial communication setup
    boost::system::error_code ec_arduino;
    sp.open("/dev/ttyACM0", ec_arduino); //connect to port
    if (ec_arduino)
        std::cout << "Could not open serial port \n";
    sp.set_option(serial_port_base::baud_rate{1000000}, ec_arduino);

    // TCP communication setup
    server s{io, 10000};

    io.run();
}