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
    try {
        config = toml::parse_file(filename);
        std::cout << "Configuration loaded successfully from " << filename << "\n";
        return true;
    }
    catch(const toml::parse_error &err) {
        std::cerr << "TOML parse error in file " << filename << ": " 
                  << err.description() << "\n";
        return false;
    }
}



// Helper function to read RTC config from a TOML table.
bdr_rtc_config readBDRConfig(const toml::table& config, const std::string& beamKey, const std::string& phaseKey) {
    bdr_rtc_config rtc;

    // bdr_reduction: assume these keys exist in the beam-specific ctrl_model.
    auto beam_node = config[beamKey];
    if (!beam_node || !beam_node.is_table()) {
        std::cerr << "Beam configuration not found for key: " << beamKey << std::endl;
        std::exit(1);
    }
    auto beam_tbl = *beam_node.as_table();

    {
        auto phase_tbl = *beam_tbl[phaseKey].as_table();
        if (auto ctrl_node = phase_tbl["ctrl_model"]; ctrl_node && ctrl_node.is_table()) {
            auto ctrl_tbl = *ctrl_node.as_table();


            // state 
            rtc.state.DM_flat = ctrl_tbl["DM_flat"] ? ctrl_tbl["DM_flat"].value_or(std::string("")) : "";
            rtc.state.LO = ctrl_tbl["LO"] ? ctrl_tbl["LO"].value_or(0) : 0;
            rtc.state.controller_type = ctrl_tbl["controller_type"] ? ctrl_tbl["controller_type"].value_or(std::string("")) : "";
            rtc.state.inverse_method = ctrl_tbl["inverse_method"] ? ctrl_tbl["inverse_method"].value_or(std::string("")) : "";
            rtc.state.auto_close = ctrl_tbl["auto_close"] ? ctrl_tbl["auto_close"].value_or(int(0)) : 0;
            rtc.state.auto_open = ctrl_tbl["auto_open"] ? ctrl_tbl["auto_open"].value_or(int(1)): 1;
            rtc.state.auto_tune = ctrl_tbl["auto_tune"] ? ctrl_tbl["auto_tuen"].value_or(int(0)) : 0;
            rtc.state.simulation_mode = 0 ; 
            
            // reduction products 
            rtc.reduction.bias = convertTomlArrayToEigenMatrix(*ctrl_tbl["bias"].as_array(),rtc.reduction.bias);
            rtc.reduction.dark = convertTomlArrayToEigenMatrix(*ctrl_tbl["dark"].as_array(),rtc.reduction.dark);
            // For bias_dm and dark_dm, these are computed later from I2A.

            // pixels 
            rtc.pixels.pupil_coords = convertTomlArrayToEigenMatrix(*ctrl_tbl["pupil_coords"].as_array(),rtc.pixels.pupil_coords); // r1,r2,c1,c2 cropped pupil coordinates in global frame
            rtc.pixels.pupil_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["pupil_pixels"].as_array(),rtc.pixels.pupil_pixels); // pupil pixels (local cropped frame)
            rtc.pixels.bad_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["bad_pixels"].as_array(),rtc.pixels.bad_pixels); // bad pixels (local cropped frame)
            rtc.pixels.interior_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["interior_pixels"].as_array(),rtc.pixels.interior_pixels); // strict interior (no boundary) pupil pixels (local cropped frame)
            rtc.pixels.secondary_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["secondary_pixels"].as_array(),rtc.pixels.secondary_pixels); // secondary obstrcution shaddow pixels (local cropped frame)
            rtc.pixels.exterior_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["exterior_pixels"].as_array(),rtc.pixels.exterior_pixels); // pixels exterior to pupil with high diffraction from phasemask (local cropped frame)

            // Matricies
            rtc.matrices.szm = ctrl_tbl["szm"].value_or(0.0);
            rtc.matrices.sza = ctrl_tbl["sza"].value_or(0.0);
            rtc.matrices.szp = ctrl_tbl["szp"].value_or(0.0);

            rtc.matrices.I2A = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2A"].as_array(),rtc.matrices.I2A);
            rtc.matrices.I2M = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M"].as_array(),rtc.matrices.I2M);
            rtc.matrices.I2M_LO = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M_LO"].as_array(),rtc.matrices.I2M_LO);
            rtc.matrices.I2M_HO = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M_HO"].as_array(),rtc.matrices.I2M_HO);
            rtc.matrices.M2C = convertTomlArrayToEigenMatrix(*ctrl_tbl["M2C"].as_array(),rtc.matrices.M2C);
            rtc.matrices.M2C_LO = convertTomlArrayToEigenMatrix(*ctrl_tbl["M2C_LO"].as_array(),rtc.matrices.M2C_LO);
            rtc.matrices.M2C_HO = convertTomlArrayToEigenMatrix(*ctrl_tbl["M2C_HO"].as_array(),rtc.matrices.M2C_HO);
            rtc.matrices.I2rms_sec = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2rms_sec"].as_array(),rtc.matrices.I2rms_sec);
            rtc.matrices.I2rms_ext = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2rms_ext"].as_array(),rtc.matrices.I2rms_ext);

            // reference intensities
            rtc.reference_pupils.I0 = convertTomlArrayToEigenMatrix(*ctrl_tbl["I0"].as_array(),rtc.reference_pupils.I0);
            rtc.reference_pupils.N0 = convertTomlArrayToEigenMatrix(*ctrl_tbl["N0"].as_array(),rtc.reference_pupils.N0);
            rtc.reference_pupils.norm_pupil = convertTomlArrayToEigenMatrix(*ctrl_tbl["norm_pupil"].as_array(),rtc.reference_pupils.norm_pupil);

            // method in reference_pupils to project them onto registered DM pixels and fill rtc.reference_pupils.norm_pupil_dm, rtc.reference_pupils.I0_dm
            rtc.reference_pupils.project_to_dm( rtc.matrices.I2A ); 
            // same for darks and bias (put on DM)
            rtc.reduction.project_to_dm( rtc.matrices.I2A );

            // limits 
            rtc.limits.open_on_strehl_limit = ctrl_tbl["upper_limit"].value_or(0.0);
            rtc.limits.open_on_flux_limit = ctrl_tbl["lower_limit"].value_or(0.0);
            rtc.limits.open_on_dm_limit = ctrl_tbl["lower_limit"].value_or(0.0);
            rtc.limits.LO_offload_limit = ctrl_tbl["lower_limit"].value_or(0.0);

            // cam config when building interaction matrix 
            if (auto cam_node = ctrl_tbl["camera_config"]; cam_node && cam_node.is_table()) {
                auto cam_tbl = *cam_node.as_table();

                rtc.cam.fps = cam_tbl["fps"].value_or(std::string(""));
                rtc.cam.gain = cam_tbl["gain"].value_or(std::string(""));
                rtc.cam.testpattern = cam_tbl["testpattern"].value_or(std::string(""));

                rtc.cam.fps  = cam_tbl["fps"].value_or(std::string(""));
                rtc.cam.gain  = cam_tbl["gain"].value_or(std::string(""));
                rtc.cam.testpattern = cam_tbl["testpattern"].value_or(std::string(""));
                rtc.cam.bias = cam_tbl["bias"].value_or(std::string(""));
                rtc.cam.flat = cam_tbl["flat"].value_or(std::string(""));
                rtc.cam.imagetags = cam_tbl["imagetags"].value_or(std::string(""));
                rtc.cam.led = cam_tbl["led"].value_or(std::string(""));
                rtc.cam.events = cam_tbl["events"].value_or(std::string(""));
                rtc.cam.extsynchro = cam_tbl["extsynchro"].value_or(std::string(""));
                rtc.cam.rawimages = cam_tbl["rawimages"].value_or(std::string(""));
                rtc.cam.cooling = cam_tbl["cooling"].value_or(std::string(""));
                rtc.cam.mode = cam_tbl["mode"].value_or(std::string(""));
                rtc.cam.resetwidth = cam_tbl["resetwidth"].value_or(std::string(""));
                rtc.cam.nbreadworeset = cam_tbl["nbreadworeset"].value_or(std::string(""));
                rtc.cam.cropping = cam_tbl["cropping"].value_or(std::string(""));
                rtc.cam.cropping_columns = cam_tbl["cropping_columns"].value_or(std::string(""));
                rtc.cam.cropping_rows = cam_tbl["cropping_rows"].value_or(std::string(""));
                rtc.cam.aduoffset = cam_tbl["aduoffset"].value_or(std::string(""));

        }
    }
    }

    // init telemetry 
    rtc.telem = bdr_telem();

    // write 


    // Validate the master configuration.
    rtc.validate();
    return rtc;
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
