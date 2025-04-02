#define TOML_IMPLEMENTATION
#include "baldr.h"
#include <commander/commander.h>
#include <math.h> // For old-style C mathematics if needed.
// Commander struct definitions for json. This is in a separate file to keep the main code clean.
#include "commander_structs.h"

//----------Globals-------------------

// The input configuration
toml::table config;
int servo_mode;

// Servo parameters. These are the parameters that will be adjusted by the commander

// Mutex for the RTC.
std::mutex rtc_mutex;

// The C-red subarray
IMAGE subarray;
//----------commander functions from here---------------

// Load a configuraiton file. Return True if successful.
bool load_configuration(std::string filename) {

    return true;
}


COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("load_configuration", load_configuration, "Load a configuration file. Return true if successful.", 
        "filename"_arg="def.toml");
 }

int main(int argc, char* argv[]) {

    // Read in the configuration file
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config file>.toml [options]" << std::endl;
        return 1;
    } else {
        config = toml::parse_file(argv[1]);
        std::cout << "Configuration file read: "<< config["name"] << std::endl;
    }

    // !!! This C-Red image should likely come from the toml
    ImageStreamIO_openIm(&subarray, "CRed");

    // Start the main RTC and telemetry threads. 
    std::thread rtc_thread(rtc);
    std::thread telemetry_thread(telemetry);

    // Initialize the commander server and run it
    commander::Server s(argc, argv);
    s.run();

    // Join the fringe-tracking thread
    servo_mode = SERVO_STOP;
    rtc_thread.join();
    telemetry_thread.join();
}
