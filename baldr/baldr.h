#include <ImageStreamIO.h>
#include <stdlib.h>
#include <iostream>
#define TOML_HEADER_ONLY 0
#include <toml.hpp>
#include <mutex>
#include <thread>

//----------Defines-----------

#define N_TEL 4 // Number of telescopes
#define N_BL 6  // Number of baselines
#define N_CP 4  // Number of closure phases

// Different types of servo modes.
#define SERVO_PID 0 // PID servo mode
#define SERVO_STOP 2

//----------Constant Arrays-----------

//----- Structures and typedefs------

//-------Commander structs-------------
// An encoded 2D image in row-major form.
// If this is to be useful, then the heimdallr EncodedImage
// should become a library.
struct EncodedImage
{
    unsigned int szx, szy;
    std::string type;
    std::string message;
};
//-------End of Commander structs------

// -------- Extern global definitions ------------
// The static initial input parameters
extern toml::table config;
extern int servo_mode;

// Servo parameters. These are the parameters that will be adjusted by the commander

// We at least need a mutex for RTC parameters.
extern std::mutex rtc_mutex;

// The C-Red Image subarray
extern IMAGE subarray;

// Main thread function for the RTC
void rtc();

// Main thread function for the telemetry
void telemetry();
