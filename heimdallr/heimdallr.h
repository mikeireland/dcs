#include <complex> 
#include <fftw3.h>
#include <ImageStreamIO.h>
#include <stdlib.h>
#include <iostream>
#include <atomic>
#define TOML_HEADER_ONLY 0
#include <toml.hpp>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <fmt/core.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <zmq.hpp>
#include <chrono>

//----------Defines-----------
#define RT_USLEEP 50 // This should never be used!
#define OPD_PER_DM_UNIT 6.0 
#define OPD_PER_PIEZO_UNIT 0.15 //Should be 0.26 

#define FT_STARTING 0
#define FT_RUNNING 1
#define FT_STOPPING 2

// Fast servo type
#define SERVO_PID 0
#define SERVO_LACOUR 1
#define SERVO_STOP 2
#define SERVO_SIMPLE 3
#define SERVO_OFF 4

// Slow (offload) servo type
#define OFFLOAD_NESTED 0
#define OFFLOAD_GD 1
#define OFFLOAD_OFF 2

// The maximum number of frames to average for group delay. Delay error in wavelength from group
// delay can be 0.4/which scales to a phasor error of 0.04, while phase error can only be 0.2
// Group delay has naturally an SNR that is 2.5 times lower, so the SNR ratio is 0.2/0.04*2.5 = 12.5
// ... this means we need 12.5^2 = ~150 times more frames to average for group delay than for
// phase delay.
#define MAX_N_GD_BOXCAR 32
#define N_TO_JUMP 10 // Number of frames to wait before checking for a phase jump
#define MAX_N_BS_BOXCAR 64   // Maximum number of frames to average for bispectrum
#define MAX_N_PS_BOXCAR 64   // Maximum number of frames to average for power spectrum
#define MAX_N_PD_BOXCAR 256 // Maximum number of frames to keep for phae delay history (phasor and phase)

#define N_DARK_BOXCAR 256 // Number of frames for the running average of the dark.

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

// These are from Lacour et al. 2019, so we'll keep the same definitions. However,
// The matrix should be an Eigen matrix for rapid multiplication later.
const Eigen::Matrix<double, N_BL, N_TEL> M_lacour = (Eigen::Matrix<double, N_BL, N_TEL>() << 
    -1, 1, 0, 0,
    -1, 0, 1, 0,
    -1, 0, 0, 1,
    0, -1, 1, 0,
    0, -1, 0, 1,
    0, 0, -1, 1).finished();
// Also, we need the pseudo-inverse of this matrix for the inverse operation.
const Eigen::Matrix<double, N_TEL, N_BL> M_lacour_dag = (Eigen::Matrix<double, N_TEL, N_BL>() << 
   -0.25,-0.25, -0.25,    0,    0,    0,
    0.25,    0,     0,-0.25,-0.25,    0,
    0,    0.25,     0, 0.25,    0,-0.25,
    0,       0,  0.25,    0, 0.25, 0.25).finished();

/* const char M[N_BL][N_TEL] = {
    {-1, 1, 0, 0},
    {-1, 0, 1, 0},
    {-1, 0, 0, 1},
    {0, -1, 1, 0},
    {0, -1, 0, 1},
    {0, 0, -1, 1}
};*/

//----- Structures and typedefs------
typedef std::complex<double> dcomp;

// As we are using Eigen and not C, we will package data from many baselines into
// a single struct
struct ControlU{
    Eigen::Vector4d dl;
    Eigen::Vector4d piezo;
    Eigen::Vector4d dm_piston;
    Eigen::Vector4d search;
    Eigen::Vector4d dl_offload;
    double search_delta, omega_dl, dit;
    unsigned int search_Nsteps, steps_to_turnaround;
    int test_beam, test_n, test_ix;
    double test_value;
};

// This is our knowledge of the per-telescope delay state. Units are all in K1 wavelengths.
struct ControlA{
    Eigen::Vector4d gd, pd, pd_phasor_boxcar_avg;
};

struct Baselines{
    Eigen::Matrix<double, N_BL, 1> gd, pd, gd_snr, pd_snr, v2_K1, v2_K2, pd_av_filtered, pd_av;
    Eigen::Matrix<dcomp, N_BL, 1> gd_phasor, pd_phasor, gd_phasor_offset;
    Eigen::Matrix<dcomp, N_BL, 1> gd_phasor_boxcar[MAX_N_GD_BOXCAR];
    Eigen::Matrix<dcomp, N_BL, 1> pd_phasor_boxcar_avg;
    Eigen::Matrix<dcomp, N_BL, 1> pd_phasor_boxcar[MAX_N_PD_BOXCAR];
    int n_gd_boxcar, ix_gd_boxcar, n_pd_boxcar, ix_pd_boxcar;
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
    double gd_gain;
    // Gain when operating GD only in steps - strictly not part of PID.
    double offload_gd_gain; 
};

//-------Commander structs-------------
// An encoded 2D image in row-major form.
struct EncodedImage
{
    unsigned int szx, szy;
    std::string type;
    std::string message;
};

// The status, encoded as std::vector<double> for 
// key variables.
struct Status
{
    std::vector<double> gd_bl, pd_bl;
    std::vector<double> gd_tel, pd_tel;
    std::vector<double> gd_snr, pd_snr;
    std::vector<double> closure_phase_K1, closure_phase_K2;
    std::vector<double> v2_K1, v2_K2;
    std::vector<double> dl_offload, dm_piston;
    std::vector<double> pd_av, pd_av_filtered;
    int test_ix, test_n;
};
//-------End of Commander structs------

// -------- Extern global definitions ------------
extern IMAGE DMs[N_TEL];
extern IMAGE master_DMs[N_TEL];
// The statit initial input parameters
extern toml::table config;

// Servo parameters. These are the parameters that will be adjusted by the commander
extern int servo_mode, offload_mode;
extern uint offload_time_ms;
extern PIDSettings pid_settings;
extern ControlU control_u;
extern ControlA control_a;
extern Baselines baselines;
extern Bispectrum bispectra_K1[N_CP];
extern Bispectrum bispectra_K2[N_CP];
extern double gd_to_K1;

// Generally, we either work with beams or baselines, so have a separate lock for each.
extern std::mutex baseline_mutex, beam_mutex;
extern std::atomic<bool> zero_offload;
// DL offload variables
extern bool keep_offloading;
extern int offloads_to_do;
extern std::string delay_line_type;
extern Eigen::Vector4d search_offset;

// ForwardFt class
class ForwardFt {   
public:
    // Save dark frames as an atomic variable
    std::atomic<bool> save_dark_frames;
    
    // Count of the frame number that has been processed
    long unsigned int cnt=0;
    
    // Count of the number of errors
    int nerrors=0;

    // The Fourier transformed image.
    fftw_complex *ft;

    /// The power spectrum of the image, and the array to boxcar average.
    double *power_spectra[MAX_N_PS_BOXCAR];
    double *power_spectrum;
    double *subim, *subim_av, *dark;
    double *subim_boxcar[N_DARK_BOXCAR];
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
    double *window;
    fftw_plan plan;
    std::thread thread; 
    int mode=FT_STARTING;
    void loop();
};

// Main thread function for fringe tracking.
void fringe_tracker();

// Seeting the delay lines (needed form the main thread and from the commander)
void set_delay_lines(Eigen::Vector4d dl);

//The forward Fourier transforms
extern ForwardFt *K1ft, *K2ft;

// Delay line offloads
void set_delay_lines(Eigen::Vector4d dl);
void add_to_delay_lines(Eigen::Vector4d dl);
void set_delay_line(int dl, double value);
void dl_offload();
void start_search(uint search_dl_in, double start, double stop, double rate, int search_dt_ms, double search_snr_threshold);