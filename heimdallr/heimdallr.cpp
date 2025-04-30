#define TOML_IMPLEMENTATION
#include "heimdallr.h"
#include <commander/commander.h>
#include <math.h>
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
void linear_search(uint beam, double start, double stop, double rate, std::string actuator) {
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
    baselines.gd_snr = Eigen::VectorXd::Zero(N_TEL);
    baselines.pd_snr = Eigen::VectorXd::Zero(N_TEL);
    baseline_mutex.unlock();

    // Start the search.
    start_search(beam, start,  stop, rate);
    fmt::print("Starting search for beam {} from {} to {} at rate {} with actuator {}\n", 
        beam, start, stop, rate, actuator);
    return;
}

// Set the servo mode
void set_servo_mode(std::string mode) {
    if (mode == "off") {
        servo_mode = SERVO_OFF;
    } else if (mode == "simple") {
        servo_mode = SERVO_SIMPLE;
    } else if (mode == "pid") {
        servo_mode = SERVO_PID;
    } else if (mode == "lacour") {
        servo_mode = SERVO_LACOUR;
    } else {
        std::cout << "Servo mode not recognised" << std::endl;
        return;
    }
    // Reset the control_u parameters
    control_u.dl.setZero();
    control_u.piezo.setZero();
    control_u.dm_piston.setZero();
    std::cout << "Servo mode updated to " << servo_mode << std::endl;
    return;
}

// Set the offload time
void set_offload_time(uint time) {
    if (time < 100 || time > 10000) {
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

void set_offset_gain(double gain) {
    pid_settings.mutex.lock();
    pid_settings.pd_offset_gain = gain;
    pid_settings.mutex.unlock();
}

void set_delay_line_type(std::string type) {
    if (type == "piezo") {
        delay_line_type = "piezo";
    } else if (type == "hfo") {
        delay_line_type = "hfo";
    } else {
        std::cout << "Delay line type not recognised" << std::endl;
        return;
    }
    std::cout << "Delay line type updated to " << delay_line_type << std::endl;
}

Status get_status() {
    Status status;
    // Get the status of the system. This is a simple struct with the
    // values we want to send back to the commander.
    // We have to initialise everything to zero or we get a core dump!
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
    status.pd_offset = std::vector<double>(N_TEL);
    status.dl_offload = std::vector<double>(N_TEL);
    status.dm_piston = std::vector<double>(N_TEL);
    status.pd_av = std::vector<double>(N_BL);
    status.pd_av_filtered = std::vector<double>(N_BL);

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
        status.pd_offset[i] = control_a.pd_offset(i);
        status.dm_piston[i] = control_u.dm_piston(i);
        status.dl_offload[i] = control_u.dl_offload(i);
    }
    for (int i = 0; i < N_CP; i++) {
        status.closure_phase_K1[i] = bispectra_K1[i].closure_phase;
        status.closure_phase_K2[i] = bispectra_K2[i].closure_phase;
    }
    return status;
}

COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("linear_search", linear_search, "Execute a linear fringe search on a single beam.", 
        "beam"_arg, "start"_arg, "stop"_arg, "rate"_arg=1.0, "actuator"_arg="HFO");
    m.def("get_ps", get_ps, "Get the power spectrum in 2D", "filter"_arg="K1");
    m.def("servo", set_servo_mode, "Set the servo mode", "mode"_arg="off");
    m.def("offload", set_offload_mode, "Set the offload (slow servo) mode", "mode"_arg="off");
    m.def("offload_time", set_offload_time, "Set the offload time in ms", "time"_arg=1000);
    m.def("set_search_offset", set_search_offset, "Set the search offset in microns", 
        "offset"_arg=std::vector<double>(N_TEL, 0.0));
    m.def("dark", save_dark, "Save the dark frames");
    m.def("delay_line", set_delay_line, "Set a delay line value in microns", 
        "beam"_arg, "value"_arg=0.0);
    m.def("status", get_status, "Get the status of the system");
    m.def("gain", set_gain, "Set the gain for the servo loop", "gain"_arg=0.0);
    m.def("ggain", set_ggain, "Set the gain for the GD servo loop", "gain"_arg=0.0);
    m.def("dl_type", set_delay_line_type, "Set the delay line type", "type"_arg="piezo");
    m.def("offset_gain", set_offset_gain, "Set the phase delay offset gain", "gain"_arg=0.0);
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

    // Initialise the DMs
    for (int i = 0; i < N_TEL; i++) {
        ImageStreamIO_openIm(&DMs[i], ("dm" + std::to_string(i+1) + "disp04").c_str());
        ImageStreamIO_openIm(&master_DMs[i], ("dm" + std::to_string(i+1)).c_str());
    }

    // Initialise the two forward Fourier transform objects, 
    ImageStreamIO_openIm(&K1, "hei_k1");
    ImageStreamIO_openIm(&K2, "hei_k2");
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
    K1ft->stop();
    K2ft->stop();
}
