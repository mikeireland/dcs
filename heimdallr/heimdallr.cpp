#define TOML_IMPLEMENTATION
#include "heimdallr.h"
#include <commander/commander.h>
#include <math.h>
#include <unistd.h>
// Commander struct definitions for json. This is in a separate file to keep the main code clean.
#include "commander_structs.h"

extern "C" {
#include <b64/cencode.h> // Base64 encoding, in C so Frantz can see how it works.
}
//----------Globals-------------------
// The input configuration
toml::table config;

// Servo parameters. These are the parameters that will be adjusted by the commander
int servo_mode=SERVO_OFF;
int offload_mode=OFFLOAD_OFF;
uint offload_time_ms=1000;
PIDSettings pid_settings;
ControlU control_u;
ControlA control_a;
Baselines baselines;
Bispectrum bispectra_K1[N_CP];
Bispectrum bispectra_K2[N_CP];

// Generally, we either work with beams or baselines, so have a separate lock for each.
std::mutex baseline_mutex, beam_mutex;

std::atomic<bool> zero_offload=false; // Atomic variable to zero the dl_offload

//The forward Fourier transforms
ForwardFt *K1ft, *K2ft;

// Offload globals
bool keep_offloading = true;
int offloads_to_do = 0;
Eigen::Vector4d search_offset = Eigen::Vector4d::Zero();
std::string delay_line_type="piezo";

IMAGE DMs[N_TEL];
IMAGE master_DMs[N_TEL];

// Utility functions

// Based on https://sourceforge.net/p/libb64/git/ci/master/tree/examples/c-example1.c
// If bandwith is an issue, we could compress the data before encoding it.
std::string encode(const char* input, unsigned int size)
{
	/* set up a destination buffer large enough to hold the encoded data */
    // print the size of the input
    std::cout << "Size of input: " << size << std::endl;
	//char* output = (char*)malloc(size*4/3 + 4); /* large enough */
	char* output = (char*)malloc(size*2); /* large enough */
	/* keep track of our encoded position */
	char* c = output;
	/* store the number of bytes encoded by a single call */
	int cnt = 0;
	/* we need an encoder state */
	base64_encodestate s;
	
	/*---------- START ENCODING ----------*/
	/* initialise the encoder state */
	base64_init_encodestate(&s);
	/* gather data from the input and send it to the output */
	cnt = base64_encode_block(input, size, c, &s);
	c += cnt;
	/* since we have encoded the entire input string, we know that 
	   there is no more input data; finalise the encoding */
	cnt = base64_encode_blockend(c, &s);
	c += cnt;
	/*---------- STOP ENCODING  ----------*/
	
    /* we want to convert to a C++ string, so null-terminate */
	*c = 0;
    // Convert the char* to a string
    std::string output_str(output);

    // Free the memory
    free(output);

    return output_str;
}

//----------commander functions from here---------------
void linear_search(uint beam, double start, double stop, double rate, uint search_dt_ms, double search_snr_threashold) {
    if (beam >= N_TEL) {
        std::cout << "Beam number (arg 0) out of range" << std::endl;
        return;
    }
    beam_mutex.lock();
    
    // Set the delay line to the start position
    set_delay_line(beam, start);

    //!!! Add code to set the DM piston offset to zero?

    beam_mutex.unlock();
    usleep(DELAY_MOVE_USEC); // Wait for the delay line to move
    // Set the SNR values to zero.
    baseline_mutex.lock();
    baselines.gd_snr = Eigen::VectorXd::Zero(N_BL); 
    baselines.pd_snr = Eigen::VectorXd::Zero(N_BL);
    baseline_mutex.unlock();

    // Start the search.
    start_search(beam, start,  stop, rate, search_dt_ms, search_snr_threashold);
    fmt::print("Starting search for beam {} from {} to {} at rate {}\n", 
        beam, start, stop, rate);
    return;
}

// Set the servo mode
void set_servo_mode(std::string mode) {
    if (mode == "off") {
        servo_mode = SERVO_OFF;
    } else if (mode == "simple") {
        servo_mode = SERVO_SIMPLE;
    } else if (mode == "fight") {
        servo_mode = SERVO_FIGHT;
    } else if (mode == "lacour") {
        servo_mode = SERVO_LACOUR;
    } else if (mode == "on") {
        servo_mode = SERVO_LACOUR;
    } else {
        std::cout << "Servo mode not recognised" << std::endl;
        return;
    }
    // Reset the control_u parameters
    control_u.dl.setZero();
    control_u.piezo.setZero();
    control_u.dm_piston.setZero();
    control_u.search_Nsteps=0;
    std::cout << "Servo mode updated to " << servo_mode << std::endl;
    return;
}

// Set the offload time
void set_offload_time(uint time) {
    if (time < 10 || time > 10000) {
        std::cout << "Offload time out of range (0.01 to 10s)" << std::endl;
        return;
    }
    offload_time_ms = time;
    std::cout << "Offload time updated to " << offload_time_ms << " ms" << std::endl;
    return;
}

// Set the offload mode
void set_offload_mode(std::string mode) {
    if (mode == "off") {
        offload_mode = OFFLOAD_OFF;
    } else if (mode == "nested") {
        offload_mode = OFFLOAD_NESTED;
        // Reset the offload to zero.
        control_u.dl_offload.setZero();
    } else if (mode == "gd") {
        offload_mode = OFFLOAD_GD;
    } else {
        std::cout << "Offload mode not recognised" << std::endl;
        return;
    }
    std::cout << "Offload mode updated to " << offload_mode << std::endl;
    return;
}

// Set the delay line offsets (from the servo loop)
void set_search_offset(std::vector<double> offset_in_microns) {
    for (uint i = 0; i < N_TEL; i++) {
        if (i < offset_in_microns.size()) {
            search_offset(i) = offset_in_microns[i];
        } else {
            search_offset(i) = 0.0;
        }
    }
    std::cout << "Search offset updated to " << search_offset.transpose() << std::endl;
}

// Get the delay line offsets (from the servo loop)
std::vector<double> get_search_offset(void) {
    std::vector<double> offsets(N_TEL);
    for (uint i = 0; i < N_TEL; i++) {
        offsets[i] = search_offset(i);
    }
    return offsets;
}

// EncodedImage  // std::string
EncodedImage get_ps(std::string filter) {
    // We get the power spectrum for one filter. This is a 2D array.
    // !!!! Warning: This is not thread-safe - we could have a mutex
    // for each filter, but that would be overkill for now.

    ForwardFt *ft;
    bool instantaneous=false;
    std::string encoded_ps;
    if (filter == "K1") {
        ft = K1ft;
    } else if (filter == "K2") {
        ft = K2ft;
    } else if (filter == "K1i") {
        ft = K1ft;
        instantaneous=true;
    } else if (filter == "K2i") {
        ft = K2ft;
        instantaneous=true;
    } else {
        throw std::runtime_error("Filter not recognised - please edit this code for a better error response");
    }
    unsigned int sz_in_bytes = ft->subim_sz * (ft->subim_sz / 2 + 1) * sizeof(double);
    // Encoding doesn't over-write the array, so pass directly to encode.
    if (instantaneous){
        encoded_ps = encode((char*)ft->power_spectra[ft->ps_index], sz_in_bytes);
    } else {
        encoded_ps = encode((char*)ft->power_spectrum, sz_in_bytes);
    }
    EncodedImage ei = {ft->subim_sz, ft->subim_sz / 2 + 1, "double", encoded_ps};
    return ei; //encoded_ps;  
}

void save_dark() {
    // Save the dark frames for K1 and K2
    K1ft->save_dark_frames = true;
    K2ft->save_dark_frames = true;
}

void set_gain(double gain) {
    pid_settings.mutex.lock();
    pid_settings.kp = gain;
    pid_settings.mutex.unlock();
}

void set_ggain(double gain) {
    pid_settings.mutex.lock();
    pid_settings.gd_gain = gain / MAX_N_GD_BOXCAR;
    pid_settings.mutex.unlock();
}

void set_offload_gd_gain(double gain) {
    pid_settings.mutex.lock();
    pid_settings.offload_gd_gain = gain;
    pid_settings.mutex.unlock();
}

void set_delay_line_type(std::string type) {
    static const std::set<std::string> valid_types = {"piezo", "hfo", "rmn"};
    if (valid_types.count(type)) {
        delay_line_type = type;
        std::cout << "Delay line type updated to " << delay_line_type << std::endl;
    } else {
        std::cout << "Delay line type not recognised: " << type << std::endl;
    }
}

// Add setter functions for thresholds
void set_gd_threshold(double val) { gd_threshold = val; }
void set_pd_threshold(double val) { pd_threshold = val; }
void set_gd_search_reset(double val) { gd_search_reset = val; }

Status get_status() {
    Status status;
    // Get the status of the system. This is a simple struct with the
    // values we want to send back to the commander.
    // We have to initialise everything to zero or we get a core dump!
    // !!! Should also truncate number of decimals.
    status.gd_bl = std::vector<double>(N_BL);
    status.pd_bl = std::vector<double>(N_BL);
    status.gd_tel = std::vector<double>(N_TEL);
    status.pd_tel = std::vector<double>(N_TEL);
    status.gd_snr = std::vector<double>(N_BL);
    status.pd_snr = std::vector<double>(N_BL);
    status.v2_K1 = std::vector<double>(N_BL);
    status.v2_K2 = std::vector<double>(N_BL);
    status.closure_phase_K1 = std::vector<double>(N_CP);
    status.closure_phase_K2 = std::vector<double>(N_CP);
    status.dl_offload = std::vector<double>(N_TEL);
    status.dm_piston = std::vector<double>(N_TEL);
    status.pd_av = std::vector<double>(N_BL);
    status.pd_av_filtered = std::vector<double>(N_BL);
    status.test_ix = control_u.test_ix;
    status.test_n = control_u.test_n;

    // Now fill these in with the values from the control structures.
    for (int i = 0; i < N_BL; i++) {
        status.gd_bl[i] = baselines.gd(i);
        status.pd_bl[i] = baselines.pd(i);
        status.gd_snr[i] = baselines.gd_snr(i);
        status.pd_snr[i] = baselines.pd_snr(i);
        status.v2_K1[i] = baselines.v2_K1(i);
        status.v2_K2[i] = baselines.v2_K2(i);
        status.pd_av[i] = baselines.pd_av(i);
        status.pd_av_filtered[i] = baselines.pd_av_filtered(i);
    }
    for (int i = 0; i < N_TEL; i++) {
        status.gd_tel[i] = control_a.gd(i);
        status.pd_tel[i] = control_a.pd(i);
        status.dm_piston[i] = control_u.dm_piston(i);
        status.dl_offload[i] = last_offload(i); // not the offload increment, but the total offload!
    }
    for (int i = 0; i < N_CP; i++) {
        status.closure_phase_K1[i] = bispectra_K1[i].closure_phase;
        status.closure_phase_K2[i] = bispectra_K2[i].closure_phase;
    }
    return status;
}

void test(uint beam, double value, uint n) {
    // This is a test function that sets the DM piston to a value
    // and then waits for n seconds.
    if (beam > N_TEL) {
        std::cout << "Beam number (arg 0) out of range" << std::endl;
        return;
    }
    beam_mutex.lock();
    control_u.test_beam = beam;
    control_u.test_value = value;
    control_u.test_n = n;
    control_u.test_ix = 0;
    beam_mutex.unlock();   
}

// Zero the Group Delay offsets, once a fringe peak is found.
void zero_gd_offsets(void){
    baseline_mutex.lock();
    for (int bl=0; bl<N_BL; bl++)
        // Set the offsets to the group delay
        baselines.gd_phasor_offset(bl) = std::conj(baselines.gd_phasor(bl)); 
    baseline_mutex.unlock();
}

// Return the phasor offsets to 3 decimal places
std::vector<double> get_gd_offsets(void){
    std::vector<double> gd_offsets(6);
    baseline_mutex.lock();
    for (int bl=0;bl<N_BL; bl++)
        gd_offsets[bl] = std::round(std::arg(baselines.gd_phasor_offset(bl)) * gd_to_K1 * 1000)/1000.0;
    baseline_mutex.unlock();
    return gd_offsets;
}

void zero_dl_offload(void){
    // Set the current positions of the delay lines to zero
    beam_mutex.lock();
    control_u.dl_offload.setZero();
    beam_mutex.unlock();
    // Set the atomic variable for dl_offload 
    zero_offload=true;
}

void set_search_params(double delta, uint turnaround){
    if (delta <= 0.0 || delta > 10.0){
        std::cout << "Search delta out of range (0.0 to 10.0 microns)" << std::endl;
        return;
    }
    if (turnaround < 1 || turnaround > 100){
        std::cout << "Search turnaround out of range (1 to 100 steps)" << std::endl;
        return;
    }
    beam_mutex.lock();
    control_u.search_delta = delta;
    control_u.steps_to_turnaround = turnaround;
    control_u.search_Nsteps = 0;
    beam_mutex.unlock();
    std::cout << "Search parameters updated: delta = " << delta << " microns, turnaround = " << turnaround << " steps" << std::endl;
}

bool foreground_in_place = false;

void set_foreground(int state) {
    // state==1: apply offsets, state==0: reverse offsets
    static const Eigen::Vector4d fg_offset(600.0, -200.0, 200.0, 600.0);
    if (state == 1 && !foreground_in_place) {
        add_to_delay_lines(fg_offset);
        foreground_in_place = true;
    } else if (state == 0 && foreground_in_place) {
        add_to_delay_lines(-fg_offset);
        foreground_in_place = false;
    }
}

COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("linear_search", linear_search, "Execute a linear fringe search on a single beam.", 
        "beam"_arg, "start"_arg, "stop"_arg, "rate"_arg=1.0, "search_dt_ms"_arg=200, "search_snr_threshold"_arg=10.0);
    m.def("get_ps", get_ps, "Get the power spectrum in 2D", "filter"_arg="K1");
    m.def("servo", set_servo_mode, "Set the servo mode", "mode"_arg="off");
    m.def("offload", set_offload_mode, "Set the offload (slow servo) mode", "mode"_arg="off");
    m.def("offload_time", set_offload_time, "Set the offload time in ms", "time"_arg=1000);
    m.def("set_search_offset", set_search_offset, "Set the search offset in microns. \n This is added to the search position when starting a search.", 
        "offset"_arg=std::vector<double>(N_TEL, 0.0));
    m.def("get_search_offset", get_search_offset, "Get the search offset in microns");
    m.def("dark", save_dark, "Save the dark frames");
    m.def("dl", set_delay_line, "Set a delay line value in microns", 
        "beam"_arg, "value"_arg=0.0);
    m.def("status", get_status, "Get the status of the system");
    m.def("gain", set_gain, "Set the gain for the servo loop", "gain"_arg=0.0);
    m.def("ggain", set_ggain, "Set the gain for the GD servo loop", "gain"_arg=0.0);
    m.def("offload_gd_gain", set_offload_gd_gain, "Set the gain when operating GD only in steps", "gain"_arg=0.0);
    m.def("dl_type", set_delay_line_type, "Set the delay line type", "type"_arg="piezo");
    m.def("test", test, "Make a test pattern", "beam"_arg, "value"_arg=0.0, "n"_arg=10);
    m.def("zero_gd_offsets", zero_gd_offsets, "Zero the group delay offsets i.e. track on this position");
    m.def("get_gd_offsets", get_gd_offsets, "Return the GD offsets in a format to be added to the toml file");
    m.def("zero_dl_offload", zero_dl_offload, "Set the current positions of the delay lines to zero");
    m.def("search", set_search_params, "Set the fringe tracker search parameter", 
        "delta"_arg=2.0, "turnaround"_arg=10);    
    m.def("set_gd_threshold", set_gd_threshold, "Set GD SNR threshold", "value"_arg=5.0);
    m.def("set_pd_threshold", set_pd_threshold, "Set PD SNR threshold", "value"_arg=4.5);
    m.def("set_gd_search_reset", set_gd_search_reset, "Set GD search reset threshold", "value"_arg=5.0);
    m.def("foreground", set_foreground, "Set (1) or unset (0) foreground delay line offsets", "state"_arg=1);
}

int main(int argc, char* argv[]) {
    IMAGE K1, K2;
    //Set the nice value.
    if (nice(-10)==-1) std::cout << "Re-niceing process likely didn't work. New nice value -1." << std::endl;
    // Read in the configuration file
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config file>.toml [options]" << std::endl;
        return 1;
    } else {
        config = toml::parse_file(argv[1]);
        std::cout << "Configuration file read: "<< config["name"] << std::endl;
    }

#ifndef SIMULATE
    // Initialise the DMs
    for (int i = 0; i < N_TEL; i++) {
        ImageStreamIO_openIm(&DMs[i], ("dm" + std::to_string(i+1) + "disp04").c_str());
        ImageStreamIO_openIm(&master_DMs[i], ("dm" + std::to_string(i+1)).c_str());
    }

    // Initialise the two forward Fourier transform objects
    ImageStreamIO_openIm(&K1, "hei_k1");
    ImageStreamIO_openIm(&K2, "hei_k2");
#else
    ImageStreamIO_openIm(&K1, "shei_k1");
    ImageStreamIO_openIm(&K2, "shei_k2");
    std::cout << "Simulation mode!" << std::endl;
   
#endif
    K1ft = new ForwardFt(&K1);
    K2ft = new ForwardFt(&K2);

    // Start the FFT threads
    K1ft->start();
    K2ft->start();

    // Start the main fringe-tracking thread. 
    std::thread fringe_thread(fringe_tracker);
    std::thread offloading_thread(dl_offload);
    
    // Initialize the commander server and run it
    commander::Server s(argc, argv);
    s.run();
    
    keep_offloading=false;
    offloading_thread.join();

    // Join the fringe-tracking thread
    servo_mode = SERVO_STOP;
    fringe_thread.join();

    // // Join the FFTW threads
    //K1ft->stop();
    //K2ft->stop();
}
