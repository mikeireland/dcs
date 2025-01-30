#define TOML_IMPLEMENTATION
#include "heimdallr.h"
#include <commander/commander.h>
#include <math.h>
#include <pthread.h>

//----------Globals-------------------

// The input configuration
toml::table config;

// Servo parameters. These are the parameters that will be adjusted by the commander
int servo_mode=SERVO_PID;
PIDSettings pid_settings;
ControlU control_us[N_TEL];
Baseline baselines[N_BL];

// Generally, we either work with beams or baselines, so have a separate lock for each.
pthread_mutex_t baseline_mutex, beam_mutex;

//The forward Fourier transforms
ForwardFt *K1ft, *K2ft;

//----------commander functions from here---------------
void linear_search(uint beam, double start, double stop, double rate, bool use_piezo) {
    if (beam >= N_TEL) {
        std::cout << "Beam number (arg 0) out of range" << std::endl;
        return;
    }
    pthread_mutex_lock(&beam_mutex);
    //!!! Add code to set the DM piston offset to zero

    // Move the delay line or piezo to the start position
    if (use_piezo) {
        //!!! Add code to move the piezo
    } else {
        //!!! Add code to move the delay line
    }
    pthread_mutex_unlock(&beam_mutex);
    usleep(DELAY_MOVE_USEC); // Wait for the delay line to move
    // Set the SNR values to zero.
    pthread_mutex_lock(&baseline_mutex);
    for (int i = 0; i < N_TEL-1; i++) {
        baselines[i].gd_snr=0;
        baselines[i].pd_snr=0;
    }
    pthread_mutex_unlock(&baseline_mutex);
    return;
}

COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("linear_search", linear_search, "Execute a linear fringe search on a single beam.", 
        "beam"_arg, "start"_arg, "stop"_arg, "rate"_arg=10.0, "use_piezo"_arg=false);

 }

int main(int argc, char* argv[]) {
    IMAGE K1, K2;

    // Read in the configuration file
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config file>.toml [options]" << std::endl;
        return 1;
    } else {
        config = toml::parse_file(argv[1]);
        std::cout << "Configuration file read: "<< config["name"] << std::endl;
    }

    // Initialise the two forward Fourier transform objects
    ImageStreamIO_openIm(&K1, "K1");
    ImageStreamIO_openIm(&K2, "K2");
    K1ft = new ForwardFt(&K1);
    K2ft = new ForwardFt(&K2);

    // Start the FFT threads
    K1ft->spawn();
    K2ft->spawn();

    // Start the main fringe-tracking thread. 
    pthread_t fringe_thread; 
    pthread_create(&fringe_thread, NULL, fringe_tracker, NULL);

    // Initialize the commander server and run it
    commander::Server s(argc, argv);
    s.run();

    // Join the fringe-tracking thread
    servo_mode = SERVO_STOP;
    pthread_join(fringe_thread, NULL);

    // Join the FFTW threads
    K1ft->join();
    K2ft->join();
}
