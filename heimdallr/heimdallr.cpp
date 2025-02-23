#define TOML_IMPLEMENTATION
#include "heimdallr.h"
#include <commander/commander.h>
#include <math.h>
#include <pthread.h>
extern "C" {
#include <b64/cencode.h> // Base64 encoding, in C so Frantz can see how it works.
}
//----------Globals-------------------

// The input configuration
toml::table config;

// Servo parameters. These are the parameters that will be adjusted by the commander
int servo_mode=SERVO_PID;
PIDSettings pid_settings;
ControlU control_us[N_TEL];
Baseline baselines[N_BL];
Bispectrum bispectra[N_CP];

// Generally, we either work with beams or baselines, so have a separate lock for each.
pthread_mutex_t baseline_mutex, beam_mutex;

//The forward Fourier transforms
ForwardFt *K1ft, *K2ft;

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

// Commander definitions for json
namespace nlohmann {
    template <>
    struct adl_serializer<EncodedImage> {
        static void to_json(json& j, const EncodedImage& p) {
            j = json{{"szx", p.szx}, {"szy", p.szy}, {"type", p.type}, {"message", p.message}};
        }
        // !!! If defined output only, this isn't needed...
        static void from_json(const json& j, EncodedImage& p) {
            j.at("szx").get_to(p.szx);
            j.at("szy").get_to(p.szy);
            j.at("type").get_to(p.type);
            j.at("message").get_to(p.message);

        }
    };
}

//----------commander functions from here---------------
void linear_search(uint beam, double start, double stop, double rate, std::string actuator) {
    if (beam >= N_TEL) {
        std::cout << "Beam number (arg 0) out of range" << std::endl;
        return;
    }
    pthread_mutex_lock(&beam_mutex);
    //!!! Add code to set the DM piston offset to zero

    // Move the delay line or piezo to the start position
    if (actuator=="piezo") {
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

// EncodedImage  // std::string
EncodedImage get_ps(std::string filter) {
    // We get the power spectrum for one filter. This is a 2D array.
    // !!!! Warning: This is not thread-safe - we could have a mutex
    // for each filter, but that would be overkill for now.

    ForwardFt *ft;
    if (filter == "K1") {
        ft = K1ft;
    } else if (filter == "K2") {
        ft = K2ft;
    } else {
        throw std::runtime_error("Filter not recognised - please edit this code for a better error response");
    }
    unsigned int sz_in_bytes = ft->subim_sz * (ft->subim_sz / 2 + 1) * sizeof(double);
    // Encoding doesn't over-write the array, so pass directly to encode.
    std::string encoded_ps = encode((char*)ft->power_spectrum, sz_in_bytes);
    EncodedImage ei = {ft->subim_sz, ft->subim_sz / 2 + 1, "double", encoded_ps};
    return ei; //encoded_ps;  
}

COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("linear_search", linear_search, "Execute a linear fringe search on a single beam.", 
        "beam"_arg, "start"_arg, "stop"_arg, "rate"_arg=10.0, "actuator"_arg="HFO");
    m.def("get_ps", get_ps, "Get the power spectrum in 2D", "filter"_arg="K1");
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

    // // Join the FFTW threads
    K1ft->join();
    K2ft->join();
}
