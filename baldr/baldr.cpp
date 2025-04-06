#define TOML_IMPLEMENTATION
#include "baldr.h"
#include <commander/commander.h>
#include <math.h> // For old-style C mathematics if needed.
// Commander struct definitions for json. This is in a separate file to keep the main code clean.
#include "commander_structs.h"

//----------Globals-------------------

// The input configuration
std::vector<int> beam_ids; 
std::vector<std::string> phasemasks;
std::vector<toml::table> configs;
std::vector<bdr_rtc_config> rtc_config_list;

int servo_mode;
//vector::<int> telescopes;

// Servo parameters. These are the parameters that will be adjusted by the commander

// Mutex for the RTC.
std::mutex rtc_mutex;

// The C-red subarray
IMAGE subarray;
//----------commander functions from here---------------




toml::table readConfig(const std::string &filename) {
    try {
        // Parse the TOML file and return the table.
        return toml::parse_file(filename);
    }
    catch(const toml::parse_error &err) {
        std::cerr << "TOML parse error in file " << filename << ": " 
                  << err.description() << "\n";
        std::exit(1);
    }
}


// Helper function to read RTC config from a TOML table. const std::string &filename,
bdr_rtc_config readBDRConfig(const toml::table& config, const std::string& beamKey, const std::string& phaseKey) {
    
    bdr_rtc_config rtc;
    
    // Read the TOML file.
    //toml::table tmpconfig = readConfig(filename);
    
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
            rtc.state.signal_space  =  ctrl_tbl["signal_space"] ? ctrl_tbl["signal_space"].value_or(std::string("")) : ""; 
            rtc.state.LO = ctrl_tbl["LO"] ? ctrl_tbl["LO"].value_or(0) : 0;
            rtc.state.controller_type = ctrl_tbl["controller_type"] ? ctrl_tbl["controller_type"].value_or(std::string("")) : "";
            rtc.state.inverse_method = ctrl_tbl["inverse_method"] ? ctrl_tbl["inverse_method"].value_or(std::string("")) : "";
            rtc.state.auto_close = ctrl_tbl["auto_close"] ? ctrl_tbl["auto_close"].value_or(int(0)) : 0;
            rtc.state.auto_open = ctrl_tbl["auto_open"] ? ctrl_tbl["auto_open"].value_or(int(1)): 1;
            rtc.state.auto_tune = ctrl_tbl["auto_tune"] ? ctrl_tbl["auto_tuen"].value_or(int(0)) : 0;
            rtc.state.simulation_mode = 0 ; 
            
            // reduction products 
            try{ 
                rtc.reduction.bias = convertTomlArrayToEigenMatrix(*ctrl_tbl["bias"].as_array(),rtc.reduction.bias,"bias");
                rtc.reduction.dark = convertTomlArrayToEigenMatrix(*ctrl_tbl["dark"].as_array(),rtc.reduction.dark,"dark");
                // For bias_dm and dark_dm, these are computed later from I2A.
            }catch (const std::exception& ex) {
                std::cerr << "Error processing rtc.reduction: " << ex.what() << std::endl;
                std::exit(1);
            }
            // pixels 
            try{ 
                rtc.pixels.crop_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["crop_pixels"].as_array(),rtc.pixels.crop_pixels,"crop_pixels"); // r1,r2,c1,c2 cropped pupil coordinates in global frame
                rtc.pixels.pupil_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["pupil_pixels"].as_array(),rtc.pixels.pupil_pixels,"pupil_pixels"); // pupil pixels (local cropped frame)
                rtc.pixels.bad_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["bad_pixels"].as_array(),rtc.pixels.bad_pixels,"bad_pixels"); // bad pixels (local cropped frame)
                rtc.pixels.interior_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["interior_pixels"].as_array(),rtc.pixels.interior_pixels,"interior_pixels"); // strict interior (no boundary) pupil pixels (local cropped frame)
                rtc.pixels.secondary_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["secondary_pixels"].as_array(),rtc.pixels.secondary_pixels,"secondary_pixels"); // secondary obstrcution shaddow pixels (local cropped frame)
                rtc.pixels.exterior_pixels = convertTomlArrayToEigenMatrix(*ctrl_tbl["exterior_pixels"].as_array(),rtc.pixels.exterior_pixels,"exterior_pixels"); // pixels exterior to pupil with high diffraction from phasemask (local cropped frame)
            }catch (const std::exception& ex) {
                std::cerr << "Error processing rtc.pixels: " << ex.what() << std::endl;
                std::exit(1);
            }

            //Filters, somewhat redundant in pixel space with the pixels struct - but could be used optimally in some cases, especially when projecting to DM space to apply weighting for boundaries!
            try{ 
                rtc.filters.bad_pixel_mask = convertTomlArrayToEigenMatrix(*ctrl_tbl["bad_pixel_mask"].as_array(),rtc.filters.bad_pixel_mask,"bad_pixel_mask"); 
                rtc.filters.pupil = convertTomlArrayToEigenMatrix(*ctrl_tbl["pupil"].as_array(),rtc.filters.pupil,"pupil"); 
                rtc.filters.secondary = convertTomlArrayToEigenMatrix(*ctrl_tbl["secondary"].as_array(),rtc.filters.secondary,"secondary"); 
                rtc.filters.exterior = convertTomlArrayToEigenMatrix(*ctrl_tbl["exterior"].as_array(),rtc.filters.exterior,"exterior"); 
                rtc.filters.inner_pupil_filt = convertTomlArrayToEigenMatrix(*ctrl_tbl["inner_pupil_filt"].as_array(),rtc.filters.inner_pupil_filt,"inner_pupil_filt");

            }catch (const std::exception& ex) {
                std::cerr << "Error processing rtc.pixels: " << ex.what() << std::endl;
                std::exit(1);
            }

            // Matricies
            try{
                rtc.matrices.szm = ctrl_tbl["szm"].value_or(0.0);
                rtc.matrices.sza = ctrl_tbl["sza"].value_or(0.0);
                rtc.matrices.szp = ctrl_tbl["szp"].value_or(0.0);
            
                rtc.matrices.I2A = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2A"].as_array(),rtc.matrices.I2A,"I2A");
                rtc.matrices.I2M = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M"].as_array(),rtc.matrices.I2M,"I2M");
                rtc.matrices.I2M_LO = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M_LO"].as_array(),rtc.matrices.I2M_LO,"I2M_LO");
                rtc.matrices.I2M_HO = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M_HO"].as_array(),rtc.matrices.I2M_HO,"I2M_HO");
                rtc.matrices.M2C = convertTomlArrayToEigenMatrix(*ctrl_tbl["M2C"].as_array(),rtc.matrices.M2C,"M2C");
                rtc.matrices.M2C_LO = convertTomlArrayToEigenMatrix(*ctrl_tbl["M2C_LO"].as_array(),rtc.matrices.M2C_LO,"M2C_LO");
                rtc.matrices.M2C_HO = convertTomlArrayToEigenMatrix(*ctrl_tbl["M2C_HO"].as_array(),rtc.matrices.M2C_HO,"M2C_HO");
                rtc.matrices.I2rms_sec = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2rms_sec"].as_array(),rtc.matrices.I2rms_sec,"I2rms_sec");
                rtc.matrices.I2rms_ext = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2rms_ext"].as_array(),rtc.matrices.I2rms_ext,"I2rms_ext");
            }catch(const std::exception& ex) {
                std::cerr << "Error processing rtc.matricies: " << ex.what() << std::endl;
                std::exit(1);
            }
            // reference intensities
            try{
                rtc.reference_pupils.I0 = convertTomlArrayToEigenMatrix(*ctrl_tbl["I0"].as_array(),rtc.reference_pupils.I0,"I0");
                rtc.reference_pupils.N0 = convertTomlArrayToEigenMatrix(*ctrl_tbl["N0"].as_array(),rtc.reference_pupils.N0,"NO");
                rtc.reference_pupils.norm_pupil = convertTomlArrayToEigenMatrix(*ctrl_tbl["norm_pupil"].as_array(),rtc.reference_pupils.norm_pupil,"norm_pupil");
            }catch(const std::exception& ex) {
                std::cerr << "Error processing rtc.reference_pupils " << ex.what() << std::endl;
                std::exit(1);
            }

            try{
                // method in reference_pupils to project them onto registered DM pixels and fill rtc.reference_pupils.norm_pupil_dm, rtc.reference_pupils.I0_dm
                rtc.reference_pupils.project_to_dm( rtc.matrices.I2A ); 
                // same for darks and bias (put on DM)
                rtc.reduction.project_to_dm( rtc.matrices.I2A );
                // project all the filters to get a weighted filter in dm space
                rtc.filters.project_to_dm( rtc.matrices.I2A );
            }catch(const std::exception& ex) {
                std::cerr << "Error processing projection of matricies to dm space " << ex.what() << std::endl;
                std::exit(1);
            }

            // limits float 
            rtc.limits.close_on_strehl_limit = ctrl_tbl["close_on_strehl_limit"].value_or(0.0);
            rtc.limits.open_on_strehl_limit = ctrl_tbl["open_on_strehl_limit"].value_or(0.0);
            rtc.limits.open_on_flux_limit = ctrl_tbl["open_on_flux_limit"].value_or(0.0);
            rtc.limits.open_on_dm_limit = ctrl_tbl["open_on_dm_limit"].value_or(0.0);
            rtc.limits.LO_offload_limit = ctrl_tbl["LO_offload_limit"].value_or(0.0);

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

    // init contoller 
    rtc.controller = bdr_controller(); 

    // write method , back to toml, or json?


    // Validate the master configuration.
    rtc.validate();
    return rtc;
}


// Parameterized constructor.
PIDController::PIDController(const Eigen::VectorXd& kp_in,
                             const Eigen::VectorXd& ki_in,
                             const Eigen::VectorXd& kd_in,
                             const Eigen::VectorXd& lower_limit_in,
                             const Eigen::VectorXd& upper_limit_in,
                             const Eigen::VectorXd& setpoint_in)
    : kp(kp_in),
      ki(ki_in),
      kd(kd_in),
      lower_limits(lower_limit_in),
      upper_limits(upper_limit_in),
      set_point(setpoint_in),
      ctrl_type("PID")
{
    int size = kp.size();
    if (ki.size() != size || kd.size() != size ||
        lower_limits.size() != size || upper_limits.size() != size ||
        set_point.size() != size) {
        throw std::invalid_argument("All input vectors must have the same size.");
    }
    output = Eigen::VectorXd::Zero(size);
    integrals = Eigen::VectorXd::Zero(size);
    prev_errors = Eigen::VectorXd::Zero(size);
}

// constructor that accepts a bdr_controller struct.
PIDController::PIDController(const bdr_controller& config_in)
    : PIDController(config_in.kp, config_in.ki, config_in.kd, config_in.lower_limits, config_in.upper_limits, config_in.set_point)
{
}

// Default constructor.
PIDController::PIDController()
    : kp(Eigen::VectorXd::Zero(140)),
      ki(Eigen::VectorXd::Zero(140)),
      kd(Eigen::VectorXd::Zero(140)),
      lower_limits(Eigen::VectorXd::Zero(140)),
      upper_limits(Eigen::VectorXd::Ones(140)),
      set_point(Eigen::VectorXd::Zero(140)),
      ctrl_type("PID")
{
    int size = kp.size();
    output = Eigen::VectorXd::Zero(size);
    integrals = Eigen::VectorXd::Zero(size);
    prev_errors = Eigen::VectorXd::Zero(size);
}

Eigen::VectorXd PIDController::process(const Eigen::VectorXd& measured) {
    int size = set_point.size();
    if (measured.size() != size) {
        throw std::invalid_argument("Input vector size must match setpoint size.");
    }
    if (kp.size() != size || ki.size() != size || kd.size() != size ||
        lower_limits.size() != size || upper_limits.size() != size) {
        throw std::invalid_argument("Input vectors of incorrect size.");
    }
    if (integrals.size() != size) {
        std::cout << "Reinitializing integrals, prev_errors, and output to zero with correct size.\n";
        integrals = Eigen::VectorXd::Zero(size);
        prev_errors = Eigen::VectorXd::Zero(size);
        output = Eigen::VectorXd::Zero(size);
    }
    for (int i = 0; i < size; ++i) {
        double error = measured(i) - set_point(i);
        if (ki(i) != 0.0) {
            integrals(i) += error;
            if (integrals(i) < lower_limits(i))
                integrals(i) = lower_limits(i);
            if (integrals(i) > upper_limits(i))
                integrals(i) = upper_limits(i);
        }
        double derivative = error - prev_errors(i);
        output(i) = kp(i) * error + ki(i) * integrals(i) + kd(i) * derivative;
        prev_errors(i) = error;
    }
    return output;
}

void PIDController::set_all_gains_to_zero() {
    kp = Eigen::VectorXd::Zero(kp.size());
    ki = Eigen::VectorXd::Zero(ki.size());
    kd = Eigen::VectorXd::Zero(kd.size());
}

void PIDController::reset() {
    integrals.setZero();
    prev_errors.setZero();
    output.setZero();
}

int load_configuration(std::string pth){
    std::cout << pth << std::endl;
    return 1;
}


COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    m.def("load_configuration", load_configuration, "Load a configuration file. Return true if successful.", 
        "filename"_arg="def.toml");


 }



// int main(int argc, char* argv[]) {

//     // Read in the configuration file
//     if (argc < 2) {
        
//         std::cout << "Usage: " << argv[0] << " <config file>.toml [options]" << std::endl;
//         //load_configuration(argv[0]);
//         return 1;
//     } else {

//         for 
//         config = toml::parse_file(argv[1]);
//         std::cout << "Configuration file read: "<< config["name"] << std::endl;

//     }

//     // !!! This C-Red image should likely come from the toml
//     ImageStreamIO_openIm(&subarray, "CRed");

//     // Start the main RTC and telemetry threads. 
//     std::thread rtc_thread(rtc);
//     std::thread telemetry_thread(telemetry);

//     // Initialize the commander server and run it
//     commander::Server s(argc, argv);
//     s.run();

//     // Join the fringe-tracking thread
//     servo_mode = SERVO_STOP;
//     rtc_thread.join();
//     telemetry_thread.join();
// }



// configure like ./baldr 1 H3 2 H3 3 H3 4 H3
int main(int argc, char* argv[]) {
    // We expect pairs of arguments: <beam_id> <phaseMask>
    if ((argc - 1) % 2 != 0 || argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <beam_id1> <phaseMask1> [<beam_id2> <phaseMask2> ...]" << std::endl;
        return 1;
    }
    
    // Parse command-line arguments in pairs.
    for (int i = 1; i < argc; i += 2) {
        try {
            int beam = std::stoi(argv[i]);
            std::string phase = argv[i + 1];
            beam_ids.push_back(beam);
            phasemasks.push_back(phase);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing arguments: " << e.what() << std::endl;
            return 1;
        }
    }
    
    // For each beam, compute the configuration filename and parse the TOML file.
    for (size_t i = 0; i < beam_ids.size(); ++i) {
        int beam = beam_ids[i];
        std::string filename = "baldr_config_" + std::to_string(beam) + ".toml";
        try {
            toml::table tbl = toml::parse_file(filename);
            configs.push_back(tbl);
            std::cout << "Loaded configuration for beam " << beam << " from " << filename << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing file " << filename << ": " << e.what() << std::endl;
            return 1;
        }
    }
    

    //to create a global list of bdr_rtc_config structs:
    for (size_t i = 0; i < configs.size(); ++i) {
        std::string beamKey = "beam" + std::to_string(beam_ids[i]);
        try {
            bdr_rtc_config cfg = readBDRConfig(configs[i], beamKey, phasemasks[i]);
            rtc_config_list.push_back(cfg);
            std::cout << "Initialized configuration for beam " << beam_ids[i] << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error initializing configuration for beam " << beam_ids[i] << ": " << e.what() << std::endl;
            return 1;
        }
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

    // Continue with your RTC main loop using rtc_config_list...
    std::cout << "DONE" << std::endl;
    
    return 0;
}


