#include <complex> 
#include <fftw3.h>
#include <ImageStreamIO.h>
#include <stdlib.h>
#include <iostream>
#define TOML_HEADER_ONLY 0
#include <toml.hpp>
#include <mutex>
#include <thread>
#include <Eigen/Dense>

//----------Defines-----------
#define FT_STARTING 0
#define FT_RUNNING 1
#define FT_STOPPING 2

#define SERVO_PID 0
#define SERVO_LACOUR 1
#define SERVO_STOP 2

// The maximum number of frames to average for group delay. Delay error in wavelength from group
// delay can be 0.4/which scales to a phasor error of 0.04, while phase error can only be 0.2
// Group delay has naturally an SNR that is 2.5 times lower, so the SNR ratio is 0.2/0.04*2.5 = 12.5
// ... this means we need 12.5^2 = ~150 times more frames to average for group delay than for
// phase delay.
#define MAX_N_GD_BOXCAR 128

#define MAX_N_BS_BOXCAR 256   // Maximum number of frames to average for bispectrum
#define MAX_N_PS_BOXCAR 256   // Maximum number of frames to average for power spectrum


#define N_TEL 4 // Number of telescopes
#define N_BL 6  // Number of baselines
#define N_CP 4  // Number of closure phases

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

const double M_pseudo_inverse[N_TEL][N_BL] = {
    {-0.25, -0.25, -0.25, 0, 0, 0},
    {0.25, 0, 0, -0.25, -0.25, 0},
    {0, 0.25, 0, 0.25, 0, -0.25},
    {0, 0, 0.25, 0, 0.25, 0.25}
};

const char closure2bl[N_CP][3] = {
    {0, 3, 1},
    {0, 4, 2},
    {1, 5, 2},
    {3, 5, 4}
};

// These are from Lacour et al. 2019, so we'll keep the same definitions.
const char M[N_BL][N_TEL] = {
    {-1, 1, 0, 0},
    {-1, 0, 1, 0},
    {-1, 0, 0, 1},
    {0, -1, 1, 0},
    {0, -1, 0, 1},
    {0, 0, -1, 1}
};

//----- Structures and typedefs------
typedef std::complex<double> dcomp;

struct ControlU{
    double dl;
    double piezo;
    double dm_piston;
};

// This is our knowledge of the per-telescope delay state.
struct ControlA{
    double gd;
    double pd;
    double delay;
    double pd_offset;
}

struct Baseline{
    double gd;
    double pd;
    double gd_snr;
    double pd_snr;
    dcomp gd_phasors[MAX_N_GD_BOXCAR];
    dcomp gd_phasor, pd_phasor;
    int n_gd_boxcar, ix_gd_boxcar;
};

struct Bispectrum{
    dcomp bs_phasors[MAX_N_BS_BOXCAR];
    dcomp bs_phasor;
    double closure_phase;
    int n_bs_boxcar, ix_bs_boxcar;
};

struct PIDSettings{
    std::mutex mutex;
    double kp;
    double ki;
    double kd;
    double integral;
    double dl_feedback_gain;
};

//-------Commander structs-------------
// An encoded 2D image in row-major form.
struct EncodedImage
{
    unsigned int szx, szy;
    std::string type;
    std::string message;
};
//-------End of Commander structs------

// -------- Extern global definitions ------------
// The statit initial input parameters
extern toml::table config;

// Servo parameters. These are the parameters that will be adjusted by the commander
extern int servo_mode;
extern PIDSettings pid_settings;
extern ControlU control_us[N_TEL];
extern ControlA control_as[N_TEL];
extern Baseline baselines[N_BL];
extern Bispectrum bispectra[N_CP];

// Generally, we either work with beams or baselines, so have a separate lock for each.
extern std::mutex baseline_mutex, beam_mutex;


// ForwardFt class
class ForwardFt {   
public:
    // Count of the frame number that has been processed
    long unsigned int cnt=0;
    
    // Count of the number of errors
    int nerrors=0;

    // The Fourier transformed image.
    fftw_complex *ft;

    /// The power spectrum of the image, and the array to boxcar average.
    double *power_spectra[MAX_N_PS_BOXCAR];
    double *power_spectrum;
    double power_spectrum_bias;
    double power_spectrum_inst_bias;
    int ps_index = MAX_N_PS_BOXCAR-1;

    // The size of the subimage, needed to determine which Fourier components to use.
    unsigned int subim_sz;

    // The image that contains the metadata.
    IMAGE *subarray;

    // Constructur - just needs an IMAGE to work on
    ForwardFt(IMAGE * subarray_in);
    
    // Spawn the thread that does the FFTs.
    void start();

    // Clean-up and join the FFT thread.
    void stop();
private:
    double *subim;
    double *window;
    fftw_plan plan;
    std::thread thread; 
    int mode=FT_STARTING;
    void loop();
};
// Main thread function for fringe tracking.
void fringe_tracker();

//The forward Fourier transforms
extern ForwardFt *K1ft, *K2ft;
