#include "heimdallr.h"
#include <commander/commander.h>
#include <math.h>
#include <pthread.h>
#include <iostream>

//#include <string>

//----------Defines: NB could be moved to a settings file-----------
#define SUBIM_SZ 32

#define SERVO_PID 0
#define SERVO_LACOUR 1

#define MAX_N_GD_BOXCAR 16

#define N_TEL 4 // Number of telescopes
#define N_BL 6  // Number of baselines

#define DELAY_MOVE_USEC 200000 // Time to wait for the delay line to move

//----- Structures ------
struct control_u{
    double dl;
    double piezo;
    double dm_piston;
};

struct baseline{
    double gd;
    double pd;
    double gd_snr;
    double pd_snr;
    fftw_complex gd_phasors[MAX_N_GD_BOXCAR];
    int n_gd_boxcar;
    fftw_complex gd_phasor, pd_phasor;
};

//----------Constant Arrays-----------
const char beam2baseline[N_TEL][N_TEL] = {
    {-1, 0, 1, 2},
    {0, -1, 3, 4},
    {1, 3, -1, 5},
    {2, 4, 5, -1}
};

const char baseline2beam[N_BL][2] = {
    {0, 1},
    {0, 2},
    {0, 3},
    {1, 2},
    {1, 3},
    {2, 3}
};

const char beam_baselines[N_TEL][N_TEL-1] = {
    {0, 1, 2},
    {0, 3, 4},
    {1, 3, 5},
    {2, 4, 5}
};

//----------Globals-------------------
// Fourier transform arrays and plans
double *subim_K1, *subim_K2;
fftw_complex *ft_K1, *ft_K2;
fftw_plan p_K1, p_K2, p_K1_inv, p_K2_inv;

// Servo parameters. These are the parameters that will be adjusted by the commander
int servo_mode=SERVO_PID;
struct baseline baselines[N_BL];
struct control_u control_us[N_TEL];
// Generally, we either work with beams or baselines, so have a separate lock for each.
pthread_mutex_t baseline_mutex;
pthread_mutex_t beam_mutex;

struct {
    pthread_mutex_t mutex;
    double gain;
    double dl_feedback_gain;
} pid_settings;

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
    m.def("linear_search", linear_search, "Execute a linear fringe search on a singel beam.", 
        "beam"_arg, "start"_arg, "stop"_arg, "rate"_arg=10.0, "use_piezo"_arg=false);

 }

int main(int argc, char* argv[]) {
    using namespace std;
    IMAGE *K1, *K2;

    // Initialise the two forward Fourier transform objects
    ForwardFt K1ft(K1);
    ForwardFt K2ft(K2);

    // Start the FFTW threads
    K1ft.spawn();
    K2ft.spawn();

    // Initialize the commander server
    commander::Server s(argc, argv);

    s.run();
}
