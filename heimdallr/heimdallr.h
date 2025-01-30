#include <complex> 
#include <fftw3.h>
#include <pthread.h>
#include <ImageStreamIO.h>
#include <stdlib.h>
#include <iostream>
#define TOML_HEADER_ONLY 0
#include <toml.hpp>

//----------Defines-----------
#define FT_STARTING 0
#define FT_RUNNING 1
#define FT_STOPPING 2

#define SERVO_PID 0
#define SERVO_LACOUR 1
#define SERVO_STOP 2

#define MAX_N_GD_BOXCAR 16

#define N_TEL 4 // Number of telescopes
#define N_BL 6  // Number of baselines

#define DELAY_MOVE_USEC 200000 // Time to wait for the delay line to move

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

//----- Structures and typedefs------
typedef std::complex<double> dcomp;

struct ControlU{
    double dl;
    double piezo;
    double dm_piston;
};

struct Baseline{
    double gd;
    double pd;
    double gd_snr;
    double pd_snr;
    dcomp gd_phasors[MAX_N_GD_BOXCAR];
    int n_gd_boxcar;
    dcomp gd_phasor, pd_phasor;
};

struct PIDSettings{
    pthread_mutex_t mutex;
    double gain;
    double dl_feedback_gain;
};

// -------- Extern global definitions ------------
// The statit initial input parameters
extern toml::table config;

// Servo parameters. These are the parameters that will be adjusted by the commander
extern int servo_mode;
extern PIDSettings pid_settings;
extern ControlU control_us[N_TEL];
extern Baseline baselines[N_BL];

// Generally, we either work with beams or baselines, so have a separate lock for each.
extern pthread_mutex_t baseline_mutex, beam_mutex;


// ForwardFt class
class ForwardFt {   
public:
    // Count of the frame number that has been processed
    long unsigned int cnt=0;
    
    // Count of the number of errors
    int nerrors=0;

    // The Fourier transformed image.
    fftw_complex *ft;

    // The size of the subimage, needed to determine which Fourier components to use.
    unsigned int subim_sz;

    // The image that contains the metadata.
    IMAGE *subarray;

    // Constructur - just needs an IMAGE to work on
    ForwardFt(IMAGE * subarray_in);
    
    // Spawn the thread that does the FFTs.
    void spawn();

    // Clean-up and join the FFT thread.
    void join();
private:
    double *subim;
    double *window;
    fftw_plan plan;
    pthread_t thread;
    int mode=FT_STARTING;
    static void* start(void* arg);
};
// Main thread function for fringe tracking.
void* fringe_tracker(void* arg);

//The forward Fourier transforms
extern ForwardFt *K1ft, *K2ft;
