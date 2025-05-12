#define TOML_IMPLEMENTATION
#include "baldr.h"
#include <commander/commander.h>
#include <math.h> // For old-style C mathematics if needed.
#include <zmq.hpp>
#include <fitsio.h>
#include <string>
#include <regex>
#include <filesystem> //C++ 17
// Commander struct definitions for json. This is in a separate file to keep the main code clean.
#include "commander_structs.h"
#include <atomic>
#include <condition_variable>

using json = nlohmann::json;
//----------Globals-------------------

// The input configuration
int beam_id; 
std::string phasemask;
toml::table config;
bdr_rtc_config rtc_config;

std::string observing_mode = "bright"; // bright - 12x12 (default) or faint - 6x6 pixels
std::string user_config_filename = "";
bool user_provided_config = false;      // true if user gives a manual toml file


// loop time , not nessary when using semaphores for frames - but can be useful for testing 
//float loop_time = 0.0f; // set later
//bool loop_time_override = false;

// int servo_mode;
std::atomic<int> servo_mode; 
std::atomic<int> servo_mode_LO;
std::atomic<int> servo_mode_HO;


std::string telemFormat = "fits";//"json";
std::string telem_save_path = "/home/asg/Music/";


// Servo parameters. These are the parameters that will be adjusted by the commander

// Mutex for the RTC.
std::mutex rtc_mutex;
// mutex telemetry
std::mutex telemetry_mutex;
// mutex for controller gain updates:
std::mutex ctrl_mutex;
//std::mutex ctrl_LO_mutex;//later
//std::mutex ctrl_HO_mutex;
std::atomic<bool> pause_rtc(false);           // When true, RTC loop should pause. atomic to be thread safe 
std::mutex rtc_pause_mutex;  //protect access to shared state (in this case, the pause flag and any associated state) while waiting and notifying.
std::condition_variable rtc_pause_cv; //condition variable is used to block (put to sleep) the RTC thread until a particular condition is met


IMAGE subarray; // The C-red subarray
IMAGE dm_rtc; // The DM subarray
IMAGE dm_rtc0; // The DM subarray master to post semaphores to


// foprward dec.
void pauseRTC();
void resumeRTC();

// Initialize global ZMQ variables
zmq::context_t cam_zmq_context(1);
zmq::socket_t cam_zmq_socket(cam_zmq_context, zmq::socket_type::req);
std::string cam_host_str = "tcp://172.16.8.6:6667";
bool cam_zmq_initialized = false;

// Initialize global ZMQ variables for MultiDeviceServer
zmq::context_t mds_zmq_context(1);
zmq::socket_t mds_zmq_socket(mds_zmq_context, zmq::socket_type::req);
std::string mds_host_str = "tcp://172.16.8.6:5555";
bool mds_zmq_initialized = false;


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
      lower_limits(-Eigen::VectorXd::Ones(140)),
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
        double derivative = error - prev_errors(i); // if error bigger than previous error you want to dampen output
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


//----------helper functions from here---------------

// Function to find the most recent config file for a given beam and mode (default = "bright")
std::string find_latest_config_file(int beam_id, const std::string& mode = "bright") {
    const std::string base_dir = "/usr/local/etc/baldr/rtc_config/";
    const std::string filename_prefix = "baldr_config_" + std::to_string(beam_id) + "_" + mode + "_";
    const std::string filename_suffix = ".toml";

    std::vector<std::filesystem::directory_entry> matching_files;

    for (const auto& dir_entry : std::filesystem::recursive_directory_iterator(base_dir)) {
        if (dir_entry.is_regular_file()) {
            const std::string filename = dir_entry.path().filename().string();
            if (filename.size() >= filename_prefix.size() + filename_suffix.size() &&
                filename.substr(0, filename_prefix.size()) == filename_prefix &&
                filename.substr(filename.size() - filename_suffix.size()) == filename_suffix) {
                matching_files.push_back(dir_entry);
            }
        }
    }

    if (matching_files.empty()) {
        throw std::runtime_error("No config file found for beam " + std::to_string(beam_id) + " with mode '" + mode + "'.");
    }

    // Sort newest first based on filename (descending)
    std::sort(matching_files.begin(), matching_files.end(),
        [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
            return a.path().filename().string() > b.path().filename().string();
        }
    );

    return matching_files.front().path().string();
}

// // Function to find the most recent config file for a given beam
// std::string find_latest_config_file(int beam_id) {
//     const std::string base_dir = "/usr/local/etc/baldr/rtc_config/";
//     const std::string filename_prefix = "baldr_config_" + std::to_string(beam_id) + "_";
//     const std::string filename_suffix = ".toml";

//     std::vector<std::filesystem::directory_entry> matching_files;

//     for (const auto& dir_entry : std::filesystem::recursive_directory_iterator(base_dir)) {
//         if (dir_entry.is_regular_file()) {
//             const std::string filename = dir_entry.path().filename().string();
//             if (filename.size() >= filename_prefix.size() + filename_suffix.size() &&
//                 filename.substr(0, filename_prefix.size()) == filename_prefix &&
//                 filename.substr(filename.size() - filename_suffix.size()) == filename_suffix) {
//                 matching_files.push_back(dir_entry);
//             }
//         }
//     }

//     if (matching_files.empty()) {
//         throw std::runtime_error("No config file found for beam " + std::to_string(beam_id));
//     }

//     // Sort newest first based on filename (descending)
//     std::sort(matching_files.begin(), matching_files.end(),
//         [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
//             return a.path().filename().string() > b.path().filename().string();
//         }
//     );

//     return matching_files.front().path().string();
// }


// Asgard Camera Server
void init_cam_zmq() {
    if (!cam_zmq_initialized) {
        cam_zmq_socket.connect(cam_host_str);
        cam_zmq_initialized = true;
    }
}

std::string send_cam_cmd(const std::string& command) {
    init_cam_zmq();
    std::string full_cmd = "cli \"" + command + "\"";
    cam_zmq_socket.send(zmq::buffer(full_cmd), zmq::send_flags::none);
    zmq::message_t reply;
    cam_zmq_socket.recv(reply, zmq::recv_flags::none);
    return std::string(static_cast<char*>(reply.data()), reply.size());
}


json send_cam_command(json args) {
    if (!args.is_array() || args.size() != 1) {
        return json{{"error", "Expected a single string argument [\"command\"]"}};
    }
    try {
        std::string command = args.at(0).get<std::string>();

        std::string response = send_cam_cmd(command);
        return json{{"command_sent", command}, {"camera_response", response}};
    } catch (const std::exception& ex) {
        return json{{"error", ex.what()}};
    }
}

std::string extract_value(const std::string& response) {
    std::regex re("^\"\\s*(.*?)\\\\r\\\\n");
    std::smatch match;
    if (std::regex_search(response, match, re)) {
        return match[1];
    }
    return "N/A";
}

float get_float_cam_param(const std::string& command) {
    std::string response = send_cam_cmd(command);
    std::string val_str = extract_value(response);
    try {
        return std::stof(val_str);
    } catch (...) {
        std::cerr << "Failed to convert camera response to float: " << val_str << std::endl;
        return -1.0f;
    }
}


// MDS 

void init_mds_zmq() {
    if (!mds_zmq_initialized) {
        mds_zmq_socket.connect(mds_host_str);
        mds_zmq_initialized = true;
    }
}

std::string send_mds_cmd(const std::string& message) {
    init_mds_zmq();
    mds_zmq_socket.send(zmq::buffer(message), zmq::send_flags::none);
    zmq::message_t reply;
    auto result = mds_zmq_socket.recv(reply, zmq::recv_flags::none);
    if (!result.has_value()) {
        throw std::runtime_error("Timeout or error receiving reply from MDS.");
    }
    return std::string(static_cast<char*>(reply.data()), reply.size());
}

json send_mds_command(json args) {
    if (!args.is_array() || args.size() != 1) {
        return json{{"error", "Expected a single string argument [\"command\"]"}};
    }
    try {
        std::string command = args.at(0).get<std::string>();

        std::string response = send_mds_cmd(command);
        return json{{"command_sent", command}, {"mds_response", response}};
    } catch (const std::exception& ex) {
        return json{{"error", ex.what()}};
    }
}

// Helper function to split a comma-separated string into a vector of ints.
std::vector<int> split_indices(const std::string &indices_str) {
    std::vector<int> indices;
    std::stringstream ss(indices_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (!token.empty()) {
            try {
                indices.push_back(std::stoi(token));
            } catch (const std::exception &ex) {
                std::cerr << "Error converting token '" << token << "': " << ex.what() << std::endl;
            }
        }
    }
    return indices;
}


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


// Helper function to read RTC config from a TOML table (from the baldr calibration pipeline). const std::string &filename,
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
    // Check if the specified phase mask exists.
    if (!beam_tbl.contains(phaseKey)) {
        std::cerr << "Error: Phase mask \"" << phaseKey
                  << "\" not found in configuration for beam " << beamKey << std::endl;
        throw std::runtime_error("Phase mask not found");
    }
    {
        auto phase_tbl = *beam_tbl[phaseKey].as_table();
        
        if (auto ctrl_node = phase_tbl["ctrl_model"]; ctrl_node && ctrl_node.is_table()) {
            auto ctrl_tbl = *ctrl_node.as_table();

            // state 
            rtc.state.DM_flat = ctrl_tbl["DM_flat"] ? ctrl_tbl["DM_flat"].value_or(std::string("")) : "";
            rtc.state.signal_space  =  ctrl_tbl["signal_space"] ? ctrl_tbl["signal_space"].value_or(std::string("")) : ""; 
            rtc.state.LO = ctrl_tbl["LO"] ? ctrl_tbl["LO"].value_or(0) : 0;
            rtc.state.controller_type = ctrl_tbl["controller_type"] ? ctrl_tbl["controller_type"].value_or(std::string("")) : "";
            rtc.state.inverse_method_LO = ctrl_tbl["inverse_method_LO"] ? ctrl_tbl["inverse_method_LO"].value_or(std::string("")) : "";
            rtc.state.inverse_method_HO = ctrl_tbl["inverse_method_HO"] ? ctrl_tbl["inverse_method_HO"].value_or(std::string("")) : "";
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
                rtc.reference_pupils.project_to_dm( rtc.matrices.I2A );  // does both I0 and N0
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
    //// heree
    //rtc.controller = bdr_controller(); 
    //int LO_nRows = rtc.matrices.I2M_LO.cols();//rtc_config.matrices.I2M_LO.rows();
    //int HO_nRows = rtc.matrices.I2M_HO.cols();//rtc_config.matrices.I2M_HO.rows();
    //std::cout << "LO_nRows = " << LO_nRows << ", HO_nRows = " << HO_nRows << std::endl;
    rtc.ctrl_LO_config = bdr_controller( 2 ); //LO_nRows ); 
    rtc.ctrl_HO_config = bdr_controller( 140 );//HO_nRows ); 

    std::cout << "in read in rtc.ctrl_LO_config.kp.size() = " << rtc.ctrl_LO_config.kp.size() << std::endl;

            
    // write method , back to toml, or json?


    // Validate the master configuration.
    rtc.validate();
    return rtc;
}

void new_dark_and_bias() {
    try {
        std::cout << "[capture_dark_and_bias] Starting new dark/bias capture..." << std::endl;

        // Pause the RTC loop
        pauseRTC();
        std::cout << "[capture_dark_and_bias] RTC paused." << std::endl;

        // Turn off the source
        send_mds_cmd("off SBB");
        std::cout << "[capture_dark_and_bias] Light source turned off." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Record original FPS and gain
        float original_fps = get_float_cam_param("fps raw");
        float original_gain = get_float_cam_param("gain raw");

        // Set FPS to maximum (1730 Hz) for bias measurement
        send_cam_cmd("set fps 1730");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const size_t nframes = 1000;
        size_t totalPixels = static_cast<size_t>(subarray.md->size[0]) * static_cast<size_t>(subarray.md->size[1]);

        Eigen::VectorXd bias_sum = Eigen::VectorXd::Zero(totalPixels);

        // --- Capture bias frames ---
        for (size_t i = 0; i < nframes; ++i) {
            catch_up_with_sem(&subarray, 1);
            ImageStreamIO_semwait(&subarray, 1);

            //uint16_t* raw = subarray.array.UI16;
            int32_t* raw = subarray.array.SI32;
            //Eigen::Map<const Eigen::Array<uint16_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
            Eigen::Map<const Eigen::Array<int32_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
            bias_sum += rawArr.cast<double>().matrix();
        }

        Eigen::VectorXd bias = bias_sum / static_cast<double>(nframes);

        std::cout << "[capture_dark_and_bias] Bias frames captured." << std::endl;

        // Restore original FPS
        send_cam_cmd("set fps " + std::to_string(static_cast<int>(original_fps)));
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // --- Capture dark frames ---
        Eigen::VectorXd dark_sum = Eigen::VectorXd::Zero(totalPixels);

        for (size_t i = 0; i < nframes; ++i) {
            catch_up_with_sem(&subarray, 1);
            ImageStreamIO_semwait(&subarray, 1);

            //uint16_t* raw = subarray.array.UI16;
            //Eigen::Map<const Eigen::Array<uint16_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
            int32_t* raw = subarray.array.SI32;
            Eigen::Map<const Eigen::Array<int32_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
            dark_sum += rawArr.cast<double>().matrix();
        }

        Eigen::VectorXd dark = dark_sum / static_cast<double>(nframes);

        std::cout << "[capture_dark_and_bias] Dark frames captured." << std::endl;

        // Turn the source back on
        send_mds_cmd("on SBB");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "[capture_dark_and_bias] Light source turned back on." << std::endl;

        // Compute raw dark: (dark - bias) (in ADU)
        Eigen::VectorXd raw_dark = dark - bias;

        // Save to rtc_config
        rtc_config.reduction.bias = bias;
        rtc_config.reduction.dark = raw_dark * (original_fps / original_gain); // Units: ADU/s/gain

        rtc_config.reduction.bias_dm = rtc_config.matrices.I2A * rtc_config.reduction.bias;
        rtc_config.reduction.dark_dm = rtc_config.matrices.I2A * rtc_config.reduction.dark;

        rtc_config.dark_dm_runtime = rtc_config.matrices.I2A * raw_dark;

        std::cout << "[capture_dark_and_bias] Updated reduction parameters successfully." << std::endl;

        // Resume the RTC
        resumeRTC();
        std::cout << "[capture_dark_and_bias] RTC resumed." << std::endl;

    } catch (const std::exception& ex) {
        std::cerr << "[capture_dark_and_bias] Exception occurred: " << ex.what() << std::endl;
        resumeRTC(); // Ensure we resume RTC even on failure
    }
}


void capture_dark_and_bias() {
    try {
        pauseRTC();  // Pause RTC while capturing frames
        std::cout << "[capture_dark_and_bias] RTC paused for dark and bias acquisition." << std::endl;

        new_dark_and_bias();

        resumeRTC(); // Resume RTC afterwards
        std::cout << "[capture_dark_and_bias] RTC resumed after dark and bias acquisition." << std::endl;
    }
    catch (const std::exception& ex) {
        std::cerr << "[capture_dark_and_bias] Exception caught: " << ex.what() << std::endl;
        resumeRTC(); // Try to resume even on error
    }
}


void stop_baldr() {
    // mode 2 is stoppoing
    servo_mode = SERVO_STOP;
    std::cout << "servo_mode updated to " << servo_mode << "stopping Baldr RTC" << std::endl;
}

void close_baldr_LO() {
    // mode 1 is close
    servo_mode_LO = SERVO_CLOSE;
    std::cout << "servo_mode_LO updated to " << servo_mode_LO << ". CLosing Baldr LO modes" << std::endl;
}

void close_all() {
    // mode 1 is close
    servo_mode_LO = SERVO_CLOSE;
    std::cout << "servo_mode_LO updated to " << servo_mode_LO << ". CLosing Baldr LO modes" << std::endl;
    servo_mode_HO = SERVO_CLOSE;
    std::cout << "servo_mode_HO updated to " << servo_mode_LO << ". CLosing Baldr HO modes" << std::endl;
}

void open_all() {
    // mode 1 is close
    servo_mode_LO = SERVO_OPEN;
    servo_mode_HO = SERVO_OPEN;
    std::cout << "servo_mode_LO updated to " << servo_mode_LO << ". CLosing Baldr LO modes" << std::endl;
}

void open_baldr_LO() {
    // mode 0 is open
    servo_mode_LO = SERVO_OPEN;
    std::cout << "servo_mode_LO updated to  " << servo_mode_LO << ". Opening Baldr LO modes" << std::endl;
}

void close_baldr_HO() {
    // mode 1 is close
    servo_mode_HO = SERVO_CLOSE;
    std::cout << "servo_mode_HO updated to " << servo_mode_HO << ". CLosing Baldr HO modes" << std::endl;
}

void open_baldr_HO() {
    // mode 0 is open
    servo_mode_HO = SERVO_OPEN;
    std::cout << "servo_mode_HO updated to  " << servo_mode_HO << ". Opening Baldr HO modes" << std::endl;
}



void I0_update() {
    // !!! only updates rtc_config.I0_dm_runtime !!! 
    // does not update rtc_config.reference_pupils members 
    
    if (rtc_config.telem.img_dm.empty()) {
        std::cerr << "[I0_update] Warning: telemetry buffer is empty, cannot update I0." << std::endl;
        return;
    }

    // Initialize a sum vector with zeros, same size as first vector in img_dm
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(rtc_config.telem.img_dm.front().size());

    size_t count = 0;
    for (const auto& v : rtc_config.telem.img_dm) {
        if (v.size() != sum.size()) {
            std::cerr << "[I0_update] Warning: found img_dm vector with inconsistent size, skipping." << std::endl;
            continue; // Skip badly-sized vector
        }
        sum += v;
        ++count;
    }

    if (count == 0) {
        std::cerr << "[I0_update] Error: no valid vectors found for averaging." << std::endl;
        return;
    }

    // Compute average
    rtc_config.I0_dm_runtime = sum / static_cast<double>(count);

    // Subtraction of dark and bias is already done in img_dm!!! 
 

    std::cout << "[I0_update] I0_dm_runtime updated from " << count << " samples." << std::endl;
}


void N0_update() {

    // !!! only updates rtc_config.N0_dm_runtime !!! 
    // does not update rtc_config.reference_pupils members 
    std::lock_guard<std::mutex> lock(telemetry_mutex);

    if (rtc_config.telem.img_dm.empty()) {
        std::cerr << "[N0_update] Error: telemetry img_dm buffer is empty." << std::endl;
        return;
    }

    const auto& telemetry_frames = rtc_config.telem.img_dm;
    Eigen::Index P = telemetry_frames[0].size();  // size of each img_dm vector

    if (rtc_config.filters.inner_pupil_filt_dm.size() != P) {
        std::cerr << "[N0_update] Error: inner_pupil_filt_dm size mismatch." << std::endl;
        return;
    }

    // Step 1: Average all img_dm frames
    Eigen::VectorXd avg_dm = Eigen::VectorXd::Zero(P);
    size_t count = 0;
    for (const auto& frame : telemetry_frames) {
        if (frame.size() != P) {
            std::cerr << "[N0_update] Warning: img_dm frame size mismatch, skipping." << std::endl;
            continue;
        }
        avg_dm += frame;
        ++count;
    }
    if (count == 0) {
        std::cerr << "[N0_update] Error: no valid img_dm frames found." << std::endl;
        return;
    }
    avg_dm /= static_cast<double>(count);

    // Step 2: Compute the mean inside the good pupil
    double sum_inside = 0.0;
    int count_inside = 0;
    for (Eigen::Index i = 0; i < P; ++i) {
        if (rtc_config.filters.inner_pupil_filt_dm(i) > 0.7) {
            sum_inside += avg_dm(i);
            ++count_inside;
        }
    }
    if (count_inside == 0) {
        std::cerr << "[N0_update] Error: no valid pixels inside inner pupil mask." << std::endl;
        return;
    }
    double inner_mean = sum_inside / static_cast<double>(count_inside);

    // Step 3: Replace exterior pixels
    for (Eigen::Index i = 0; i < P; ++i) {
        if (rtc_config.filters.inner_pupil_filt_dm(i) <= 0.7) {
            avg_dm(i) = inner_mean;
        }
    }

    // Step 4: Save into runtime only
    rtc_config.N0_dm_runtime = avg_dm;

    std::cout << "[N0_update] Successfully updated N0_dm_runtime based on img_dm telemetry." << std::endl;
}


// Function to update the capacity of the telemetry image ring buffer.
json set_telem_capacity(json args) {
    // Expect one argument: [new_capacity]
    if (!args.is_array() || args.size() != 1) {
        return json{{"error", "Expected an array with one element: [new_capacity]"}};
    }
    try {
        size_t newCapacity = args.at(0).get<size_t>();

        // Lock the telemetry mutex during the update.
        std::lock_guard<std::mutex> lock(telemetry_mutex);
        rtc_config.telem.setCapacity(newCapacity);
        
        std::cout << "Telemetry ring buffers capacity updated to " << newCapacity << std::endl;
        return json{{"status", "Telemetry capacity updated to " + std::to_string(newCapacity)}};
    }
    catch (const std::exception &ex) {
        std::cerr << "Error updating telemetry capacity: " << ex.what() << std::endl;
        return json{{"error", ex.what()}};
    }
}

void save_telemetry(){
    rtc_config.state.take_telemetry = 1;
}

json set_telem_save_path(json args) {
    if (!args.is_array() || args.size() != 1) {
        return json{{"error", "Expected an array with one element: [\"/desired/save/path/\"]"}};
    }
    try {
        std::string new_path = args.at(0).get<std::string>();
        if (new_path.empty()) {
            return json{{"error", "Provided path is empty."}};
        }

        // Ensure path ends with a slash '/'
        if (new_path.back() != '/') {
            new_path += '/';
        }

        telem_save_path = new_path;
        // {
        //     std::lock_guard<std::mutex> lock(telemetry_mutex);
        //     telem_save_path = new_path;
        // }

        // Optionally create the directory now if it doesn't exist
        try {
            std::filesystem::create_directories(new_path);
        } catch (const std::exception& ex) {
            return json{{"error", "Failed to create directory: " + std::string(ex.what())}};
        }

        std::cout << "[set_telem_save_path] Updated telemetry save path to: " << new_path << std::endl;
        return json{{"status", "Telemetry save path updated", "new_path", new_path}};
    }
    catch (const std::exception& ex) {
        return json{{"error", ex.what()}};
    }
}


void pauseRTC() {
    pause_rtc.store(true);
    std::cout << "RTC paused." << std::endl;
}

void resumeRTC() {
    {
        std::lock_guard<std::mutex> lock(rtc_pause_mutex);
        pause_rtc.store(false);
    }
    rtc_pause_cv.notify_all();
    std::cout << "RTC resumed." << std::endl;
}


json set_telem_save_format(json args) {
    // Check that the argument is an array with exactly one element.
    if (!args.is_array() || args.size() != 1) {
        return json{{"error", "Expected an array with one element: [\"json\" or \"fits\"]"}};
    }
    try {
        // Get the format string from the argument.
        std::string format = args.at(0).get<std::string>();
        // Convert to lowercase to allow case-insensitive matching.
        std::transform(format.begin(), format.end(), format.begin(), ::tolower);
        // Check that format is either "json" or "fits".
        if (format != "json" && format != "fits") {
            return json{{"error", "Invalid format. Please use \"json\" or \"fits\"."}};
        }
        // Update the global telemFormat variable.
        telemFormat = format;
        std::cout << "Telemetry output format updated to: " << telemFormat << std::endl;
        return json{{"status", "Telemetry format updated successfully"}};
    } catch (const std::exception& ex) {
        return json{{"error", ex.what()}};
    }
}


// This Commander function accepts one JSON parameter: an array of three items:
// [ gain_type, indices, value ]
// e.g. in interactive shell : > update_pid_param ["LO","kp", "all", 0.0] or  ["HO","ki","all",0.2] or update_pid_param ["HO","ki","1,3,5",0.2] to update gains of modes 1,3,5 
json update_pid_param(json args) {
    // Expect an array with exactly 4 elements:
    // [controller, parameter_type, indices, value]
    if (!args.is_array() || args.size() != 4) {
        return json{{"error", "Expected an array with four elements: [controller, parameter_type, indices, value]"}};
    }
    try {
        std::string controller_str = args.at(0).get<std::string>();
        std::string parameter_type = args.at(1).get<std::string>();
        std::string indices_str = args.at(2).get<std::string>();
        double value = args.at(3).get<double>();

        // Lock the PID mutex to protect concurrent access.
        std::lock_guard<std::mutex> lock(ctrl_mutex);

        // Select the appropriate PID controller from rtc_config.
        PIDController* pid_ctrl = nullptr;
        if (controller_str == "LO") {
            pid_ctrl = &rtc_config.ctrl_LO;
        } else if (controller_str == "HO") {
            pid_ctrl = &rtc_config.ctrl_HO;
        } else {
            return json{{"error", "Invalid controller type: " + controller_str + ". Must be \"LO\" or \"HO\"."}};
        }
        
        // Choose the target parameter vector.
        Eigen::VectorXd* target_vector = nullptr;
        if (parameter_type == "kp") {
            target_vector = &pid_ctrl->kp;
        } else if (parameter_type == "ki") {
            target_vector = &pid_ctrl->ki;
        } else if (parameter_type == "kd") {
            target_vector = &pid_ctrl->kd;
        } else if (parameter_type == "set_point") {
            target_vector = &pid_ctrl->set_point;
        } else if (parameter_type == "lower_limits") {
            target_vector = &pid_ctrl->lower_limits;
        } else if (parameter_type == "upper_limits") {
            target_vector = &pid_ctrl->upper_limits;
        } else {
            return json{{"error", "Invalid parameter type: " + parameter_type +
                                     ". Use one of \"kp\", \"ki\", \"kd\", \"set_point\", \"lower_limits\", or \"upper_limits\"."}};
        }
        
        int n = target_vector->size();
        if (indices_str == "all") {
            target_vector->setConstant(value);
            std::cout << "Updated all elements of " << parameter_type << " in controller " << controller_str 
                      << " to " << value << std::endl;
        } else {
            // Use your split_indices helper function to split indices_str by commas.
            std::vector<int> idxs = split_indices(indices_str);
            if (idxs.empty()) {
                return json{{"error", "No valid indices provided"}};
            }
            for (int idx : idxs) {
                if (idx < 0 || idx >= n) {
                    return json{{"error", "Index " + std::to_string(idx) + " out of range for " + parameter_type 
                                        + " vector of size " + std::to_string(n)}};
                }
                (*target_vector)(idx) = value;
            }
            std::cout << "Updated indices (" << indices_str << ") of " << parameter_type << " in controller " 
                      << controller_str << " with value " << value << std::endl;
        }
        return json{{"status", "success"}};
    }
    catch (const std::exception &ex) {
        return json{{"error", ex.what()}};
    }
}

// relative incrememnt of gains 
json dg(json args) {
    // Expect an array with exactly 3 elements: [controller, parameter_type, increment]
    if (!args.is_array() || args.size() != 3) {
        return json{{"error", "Expected an array with three elements: [controller, parameter_type, increment]"}};
    }
    try {
        std::string controller_str = args.at(0).get<std::string>();
        std::string parameter_type = args.at(1).get<std::string>();
        double increment_value = args.at(2).get<double>();

        // Lock the PID mutex to protect concurrent access
        std::lock_guard<std::mutex> lock(ctrl_mutex);

        // Select appropriate PID controller
        PIDController* pid_ctrl = nullptr;
        if (controller_str == "LO") {
            pid_ctrl = &rtc_config.ctrl_LO;
        } else if (controller_str == "HO") {
            pid_ctrl = &rtc_config.ctrl_HO;
        } else {
            return json{{"error", "Invalid controller type: " + controller_str + ". Must be \"LO\" or \"HO\"."}};
        }

        // Choose the target parameter vector
        Eigen::VectorXd* target_vector = nullptr;
        if (parameter_type == "kp") {
            target_vector = &pid_ctrl->kp;
        } else if (parameter_type == "ki") {
            target_vector = &pid_ctrl->ki;
        } else if (parameter_type == "kd") {
            target_vector = &pid_ctrl->kd;
        } else {
            return json{{"error", "Invalid parameter type: " + parameter_type +
                                     ". Must be \"kp\", \"ki\", or \"kd\"."}};
        }

        // Increment all elements
        *target_vector += Eigen::VectorXd::Constant(target_vector->size(), increment_value);

        std::cout << "[increment_pid_param] Incremented " << parameter_type
                  << " in controller " << controller_str
                  << " by " << increment_value << std::endl;

        return json{{"status", "Increment applied successfully"}};
    }
    catch (const std::exception& ex) {
        return json{{"error", ex.what()}};
    }
}

json print_pid_attribute(json args) {
    // Expect exactly two elements: [controller, attribute]
    if (!args.is_array() || args.size() != 2) {
        return json{{"error", "Expected an array with two elements: [controller, attribute]"}};
    }
    
    try {
        std::string controller_str = args.at(0).get<std::string>();
        std::string attribute = args.at(1).get<std::string>();
        
        // Select the appropriate PID controller from rtc_config.
        PIDController* ctrl = nullptr;
        if (controller_str == "LO") {
            ctrl = &rtc_config.ctrl_LO;
        } else if (controller_str == "HO") {
            ctrl = &rtc_config.ctrl_HO;
        } else {
            return json{{"error", "Invalid controller specified. Must be \"LO\" or \"HO\""}};
        }

        // lock it while we read 
        std::lock_guard<std::mutex> lock(ctrl_mutex);


        // Helper lambda to convert an Eigen::VectorXd to a std::vector<double>.
        auto eigen_to_vector = [](const Eigen::VectorXd &v) -> std::vector<double> {
            return std::vector<double>(v.data(), v.data() + v.size());
        };
        

        if (attribute == "kp") {
            return json{{"kp", eigen_to_vector(ctrl->kp)}};
        } else if (attribute == "ki") {
            return json{{"ki", eigen_to_vector(ctrl->ki)}};
        } else if (attribute == "kd") {
            return json{{"kd", eigen_to_vector(ctrl->kd)}};
        } else if (attribute == "lower_limits") {
            return json{{"lower_limits", eigen_to_vector(ctrl->lower_limits)}};
        } else if (attribute == "upper_limits") {
            return json{{"upper_limits", eigen_to_vector(ctrl->upper_limits)}};
        } else if (attribute == "set_point") {
            return json{{"set_point", eigen_to_vector(ctrl->set_point)}};
        } else if (attribute == "output") {
            return json{{"output", eigen_to_vector(ctrl->output)}};
        } else if (attribute == "integrals") {
            return json{{"integrals", eigen_to_vector(ctrl->integrals)}};
        } else if (attribute == "prev_errors") {
            return json{{"prev_errors", eigen_to_vector(ctrl->prev_errors)}};
        } else {
            return json{{"error", "Unknown attribute: " + attribute}};
        }
    } catch (const std::exception &ex) {
        return json{{"error", ex.what()}};
    }
}

Eigen::VectorXd expand_to_144(const Eigen::VectorXd& input140) {
    if (input140.size() != 140) {
        throw std::runtime_error("Input vector must be of size 140");
    }

    Eigen::VectorXd output144(144);
    int j = 0;
    for (int i = 0; i < 144; ++i) {
        // Skip corners: top-left (0), top-right (11), bottom-left (132), bottom-right (143)
        if (i == 0 || i == 11 || i == 132 || i == 143) {
            output144(i) = 0.0;
        } else {
            output144(i) = input140(j++);
        }
    }
    return output144;
}

void build_interaction_matrix(double poke_amp = 0.05, int num_iterations = 10, double sleep_seconds = 0.01, const std::string& signal_space = "dm", const std::string& output_filename = "") {
    std::cout << "[IM] Starting interaction matrix capture with 140 modes..." << std::endl;

    // Take new dark and bias
    new_dark_and_bias();

    // Get gain and fps
    float gain = get_float_cam_param("gain");
    float fps = get_float_cam_param("fps");
    double darkscale = fps / gain;

    int nx = subarray.md->size[0];
    int ny = (subarray.md->naxis > 1) ? subarray.md->size[1] : 1;
    int totalPixels = nx * ny;
    int N_modes = 140;

    Eigen::MatrixXd IM(totalPixels, N_modes);

    for (int k = 0; k < N_modes; ++k) {
        std::cout << "[IM] Poking actuator " << k << "/" << N_modes << std::endl;

        Eigen::VectorXd cmd = Eigen::VectorXd::Zero(N_modes);
        cmd(k) = poke_amp / 2.0;

        Eigen::VectorXd padded_cmd(144);
        padded_cmd.setZero();
        int idx = 0;
        for (int i = 0; i < 144; ++i) {
            if (!(i == 0 || i == 11 || i == 132 || i == 143)) {
                padded_cmd(i) = cmd(idx++);
            }
        }

        std::vector<Eigen::VectorXd> Iplus_list, Iminus_list;

        for (int iter = 0; iter < num_iterations; ++iter) {
            // Positive poke
            updateDMSharedMemory(dm_rtc, padded_cmd);
            ImageStreamIO_sempost(&dm_rtc0, 1);
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds));

            Eigen::VectorXd Iplus = Eigen::VectorXd::Zero(totalPixels);
            for (int i = 0; i < 200; ++i) {
                catch_up_with_sem(&subarray, 1);
                ImageStreamIO_semwait(&subarray, 1);
                //uint16_t *raw = subarray.array.UI16;
                //Eigen::Map<const Eigen::Array<uint16_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
                int32_t* raw = subarray.array.SI32;
                Eigen::Map<const Eigen::Array<int32_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
                Eigen::VectorXd img = rawArr.cast<double>();
                img -= (darkscale * rtc_config.reduction.dark + rtc_config.reduction.bias);
                Iplus += img;
            }
            Iplus /= 200.0;
            Iplus_list.push_back(Iplus);

            // Negative poke
            updateDMSharedMemory(dm_rtc, -padded_cmd);
            ImageStreamIO_sempost(&dm_rtc0, 1);
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_seconds));

            Eigen::VectorXd Iminus = Eigen::VectorXd::Zero(totalPixels);
            for (int i = 0; i < 200; ++i) {
                catch_up_with_sem(&subarray, 1);
                ImageStreamIO_semwait(&subarray, 1);
                //uint16_t *raw = subarray.array.UI16;
                //Eigen::Map<const Eigen::Array<uint16_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
                int32_t* raw = subarray.array.SI32;
                Eigen::Map<const Eigen::Array<int32_t, Eigen::Dynamic, 1>> rawArr(raw, totalPixels);
                Eigen::VectorXd img = rawArr.cast<double>();
                img -= (darkscale * rtc_config.reduction.dark + rtc_config.reduction.bias);
                Iminus += img;
            }
            Iminus /= 200.0;
            Iminus_list.push_back(Iminus);
        }

        Eigen::VectorXd Iplus_mean = Eigen::VectorXd::Zero(totalPixels);
        Eigen::VectorXd Iminus_mean = Eigen::VectorXd::Zero(totalPixels);
        for (int i = 0; i < num_iterations; ++i) {
            Iplus_mean += Iplus_list[i];
            Iminus_mean += Iminus_list[i];
        }
        Iplus_mean /= static_cast<double>(num_iterations);
        Iminus_mean /= static_cast<double>(num_iterations);

        Eigen::VectorXd errsig = (Iplus_mean - Iminus_mean) * darkscale / poke_amp;
        IM.col(k) = errsig;
    }

    std::string filename = output_filename.empty() ? "/home/asg/Music/IM_test.fits" : output_filename;
    write_eigen_matrix_to_fits(IM, filename, "IM");
    std::cout << "[IM] Saved interaction matrix to " << filename << std::endl;
}




json reload_config(json args) {
    // Expect two arguments: [new_filename, new_phase_mask]. filename MUST be .toml from the Baldr calibration pipeline
    if (!args.is_array() || args.size() != 2) {
        return json{{"error", "Expected an array with two elements: [new_filename, new_phase_mask]"}};
    }
    try {
        // Extract new configuration file name and phase mask.
        std::string newFilename = args.at(0).get<std::string>();
        std::string newPhaseMask = args.at(1).get<std::string>();

        // Pause the RTC loop.
        pauseRTC();
        std::cout << "RTC paused for configuration reload." << std::endl;

        // Parse the new TOML configuration file.
        config = toml::parse_file(newFilename);
        std::cout << "Loaded new configuration file: " << newFilename << std::endl;

        // Update the phase mask globally.
        phasemask = newPhaseMask;

        // Construct the beam key based on the current beam number.
        std::string beamKey = "beam" + std::to_string(beam_id);

        // Update rtc_config using the new configuration.
        rtc_config = readBDRConfig(config, beamKey, phasemask);
        std::cout << "rtc_config updated using beam " << beamKey << " and phase mask " << phasemask << std::endl;

        // Reinitialize PID controllers using the new controller configuration.
        {
            //std::lock_guard<std::mutex> lock1(ctrl_mutex);
            //std::lock_guard<std::mutex> lock2(telemetry_mutex);
            std::scoped_lock lock(ctrl_mutex, telemetry_mutex); //C++17
            // ctrl_LO = PIDController(rtc_config.ctrl_LO_config);
            // ctrl_HO = PIDController(rtc_config.ctrl_HO_config);
            // std::cout << "PID controllers reinitialized: "
            //           << "ctrl_LO.kp.size()=" << ctrl_LO.kp.size() << ", "
            //           << "ctrl_HO.kp.size()=" << ctrl_HO.kp.size() << std::endl;
            rtc_config.initDerivedParameters();

        }

        
        //rtc_config.telem = bdr_telem();  // Reset telemetry buffers.

        // Resume the RTC loop.
        resumeRTC();
        std::cout << "RTC resumed after configuration reload." << std::endl;

        return json{{"status", "New configuration loaded successfully"}};
    }
    catch (const std::exception &ex) {
        std::cerr << "Error reloading configuration: " << ex.what() << std::endl;
        // Make sure to resume if necessary, or signal an error state.
        resumeRTC();
        return json{{"error", ex.what()}};
    }
}

/// NEed to test: poking in control space so we can build IM in closed loop 
// void closed_loop_modal_id(const std::string& space = "dm", int num_frames = 20, double delta = 0.1, const std::string& output_filename = "/home/asg/Music/modal_response.fits") {
//     std::cout << "[Modal ID] Starting closed-loop identification for modes..." << std::endl;

//     // Determine which telemetry buffer to use
//     const bool use_dm_space = (space == "dm");
//     const auto& telemetry_buffer = use_dm_space ? rtc_config.telem.img_dm : rtc_config.telem.imgs;

//     if (telemetry_buffer.empty()) {
//         std::cerr << "[Modal ID] Error: telemetry buffer is empty." << std::endl;
//         return;
//     }

//     const int N_modes_LO = rtc_config.ctrl_LO.set_point.size();
//     const int N_modes_HO = rtc_config.ctrl_HO.set_point.size();
//     const int N_total_modes = N_modes_LO + N_modes_HO;
//     const int frame_size = telemetry_buffer.front().size();

//     std::vector<Eigen::MatrixXd> response_data;

//     // Lock to modify setpoints
//     std::lock_guard<std::mutex> lock(ctrl_mutex);

//     for (int k = 0; k < N_total_modes; ++k) {
//         std::cout << "[Modal ID] Probing mode " << k << "/" << N_total_modes << std::endl;

//         PIDController& ctrl = (k < N_modes_LO) ? rtc_config.ctrl_LO : rtc_config.ctrl_HO;
//         int local_k = (k < N_modes_LO) ? k : k - N_modes_LO;

//         // Helper lambda to collect telemetry after a setpoint change
//         auto collect = [&](double sign) -> Eigen::MatrixXd {
//             ctrl.set_point(local_k) = sign * delta;
//             std::this_thread::sleep_for(std::chrono::milliseconds(500));

//             std::vector<Eigen::VectorXd> collected;
//             for (int i = 0; i < num_frames; ++i) {
//                 if (telemetry_buffer.size() < 1) {
//                     std::cerr << "[Modal ID] Warning: no telemetry available during read." << std::endl;
//                     continue;
//                 }
//                 collected.push_back(telemetry_buffer.back());
//                 std::this_thread::sleep_for(std::chrono::milliseconds(10));
//             }

//             Eigen::MatrixXd result(frame_size, collected.size());
//             for (int i = 0; i < collected.size(); ++i)
//                 result.col(i) = collected[i];
//             return result;
//         };

//         // Positive response
//         Eigen::MatrixXd pos_resp = collect(+1);
//         response_data.push_back(pos_resp);

//         // Negative response
//         Eigen::MatrixXd neg_resp = collect(-1);
//         response_data.push_back(neg_resp);

//         // Reset setpoint
//         ctrl.set_point(local_k) = 0.0;
//     }

//     // Write to FITS file (one HDU per entry)
//     fitsfile* fptr;
//     int status = 0;
//     fits_create_file(&fptr, output_filename.c_str(), &status);
//     if (status) {
//         fits_report_error(stderr, status);
//         return;
//     }

//     for (int i = 0; i < response_data.size(); ++i) {
//         const auto& mat = response_data[i];
//         long naxes[2] = {mat.rows(), mat.cols()};
//         long fpixel[2] = {1, 1};

//         if (i == 0) {
//             fits_create_img(fptr, DOUBLE_IMG, 2, naxes, &status);
//         } else {
//             fits_create_img(fptr, DOUBLE_IMG, 2, naxes, &status);
//         }

//         fits_write_pix(fptr, TDOUBLE, fpixel, naxes[0]*naxes[1], (void*)mat.data(), &status);
//     }

//     fits_close_file(fptr, &status);
//     std::cout << "[Modal ID] Saved closed-loop response data to " << output_filename << std::endl;
// }


COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    // m.def("load_configuration", load_configuration, "Load a configuration file. Return true if successful.", 
    //     "filename"_arg="def.toml");

    
    //send_cam_command ["set gain 1"]
    m.def("send_mds_command", send_mds_command,
        "Send a raw string command to the MultiDeviceServer over ZMQ.\n"
        "Usage: send_mds_command [\"your command\"] e.g. send_mds_command [\"off SBB\"]",
        "args"_arg);

    //send_mds_command ["off SBB"]
    m.def("send_cam_command", send_cam_command,
        "Send a raw string command to the Camera Server (FLI) over ZMQ.\n"
        "Usage: send_cam_command [\"your command\"]. e.g. send_cam_command [\"set gain 1\"]",
        "args"_arg);
        

    m.def("capture_dark_and_bias", capture_dark_and_bias,
            "Capture a new bias and dark frame, update reduction products, and update DM-space maps.");
            
    m.def("stop_baldr", stop_baldr,
          "stop the baldr rtc loop","mode"_arg);

    m.def("close_baldr_LO", close_baldr_LO,
          "close the LO servo loop for baldr","mode"_arg);

    m.def("open_baldr_LO", open_baldr_LO,
          "open the LO servo loop for baldr - resetting gains and flatten DM","mode"_arg);

    m.def("close_all", close_all,
          "close LO and HO servo loop for baldr","mode"_arg);

    m.def("open_all", open_all,
          "open LO and HO servo loop for baldr - resetting gains and flatten DM","mode"_arg);

    m.def("close_baldr_HO", close_baldr_HO,
          "close the HO servo loop for baldr","mode"_arg);

    m.def("open_baldr_HO", open_baldr_HO,
          "open the HO servo loop for baldr - resetting gains and flatten DM","mode"_arg);

    m.def("save_telemetry", save_telemetry,
          "dump telemetry in the circular buffer to file","mode"_arg);

    m.def("set_telem_save_path", set_telem_save_path,
        "Set the directory to save telemetry files. Usage: set_telem_save_path [\"/your/save/path/\"]",
        "args"_arg);
        
    m.def("set_telem_capacity", set_telem_capacity,
          "Set the capacity for all telemetry ring buffers (how many samples we hold). e.g: set_telem_capacity [200]",
          "args"_arg);


    m.def("I0_update", I0_update,
            "Update I0_dm_runtime by averaging the telemetry img_dm buffer (no arguments required).");
            
    m.def("N0_update", N0_update,
            "Update N0 reference: averages img telemetry, corrects exterior pixels, and projects to DM space");
            
    m.def("set_telem_save_format", set_telem_save_format,
          "Update the telemetry output format. Acceptable values: \"json\", \"fits\".\n"
          "Usage: set_telem_format [\"json\"] or set_telem_format [\"fits\"]",
          "args"_arg);


    m.def("pause_baldr_rtc", [](){
        pauseRTC();
        return std::string("RTC paused.");
    }, "Pause the RTC loop.\n\n");

    m.def("resume_baldr_rtc", [](){
        resumeRTC();
        return std::string("RTC resumed.");
    }, "Resume the RTC loop.\n\n");


    //update_pid_param ["LO","kp", "all",0.1] or update_pid_param ["HO","kp","1,3,5,42",0.1] to update gains of particular mode indicies  (1,3,5,42) to 0.1
    m.def("update_pid_param", update_pid_param,
          "Update a PID gain vector or set_point, upper_limit or lower_limit. Parameters: [mode, parameter, indices, value].\n"
          "  - mode: LO or HO"
          "  - gain_type: \"kp\", \"ki\", or \"kd\"\n"
          "  - indices: a comma-separated list of indices or \"all\"\n"
          "  - value: the new gain value (number)\n"
          "e.g. (use double quotation) update_pid_param [''LO'',''kp'',''all'', 0.0] or update_pid_param [''HO'',''kp'',''1,3,5,42'',0.1] or ",
          "args"_arg);

    //e.g: dg ["LO","ki",0.1] 
    m.def("dg", dg,
        "Increment all elements of kp, ki, or kd of the specified controller by a given increment.\n"
        "Usage: increment_pid_param [\"LO\" or \"HO\", \"kp\" or \"ki\" or \"kd\", increment]",
        "args"_arg);


    //print_pid_attribute ["HO","ki"], print_pid_attribute ["LO","kp"]
    m.def("print_pid_attribute", print_pid_attribute,
          "Print the specified attribute of a PID controller.\n"
          "Usage: print_pid_attribute [controller, attribute]\n"
          "  - controller: \"LO\" or \"HO\"\n"
          "  - attribute: one of \"kp\", \"ki\", \"kd\", \"lower_limits\", \"upper_limits\", \"set_point\", "
          "\"output\", \"integrals\", or \"prev_errors\"",
          "args"_arg);

    //build_interaction_matrix [0.05, 10, 0.1, "dm", "/home/asg/Music/IM_test.fits"]
    m.def("build_interaction_matrix",
        [](json args) {
            if (!args.is_array() || args.size() > 5) {
                return json{{"error", "Usage: build_interaction_matrix [poke_amp, num_iterations, sleep_seconds, signal_space, output_filename]"}};
            }

            double poke_amp = args.size() > 0 ? args[0].get<double>() : 0.05;
            int num_iterations = args.size() > 1 ? args[1].get<int>() : 10;
            double sleep_seconds = args.size() > 2 ? args[2].get<double>() : 0.01;
            std::string signal_space = args.size() > 3 ? args[3].get<std::string>() : "dm";
            std::string output_filename = args.size() > 4 ? args[4].get<std::string>() : "";

            build_interaction_matrix(poke_amp, num_iterations, sleep_seconds, signal_space, output_filename);

            return json{{"status", "interaction matrix construction complete"}};
            },
            "Builds interaction matrix: build_interaction_matrix [poke_amp, num_iter, sleep_s, signal_space, output_filename]",
            "args"_arg
    );

    //e.g. reload_config ["new_config.toml", "H5"]
    m.def("reload_config", reload_config,
          "Reload a new configuration file with a specified phasemask. It must be .toml \n"
          "from the baldr calibration pipeline. Also assumes the same beam_id as is currently configured.\n"
          "Usage: reload_config [\"new_filename.toml\", \"new_mask\"]",
          "args"_arg);


 }





int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 5) {
        std::cerr << "Usage: " << argv[0] << " <beam_id> <phaseMask> [mode (bright|faint)] [optional config_filename.toml]" << std::endl;
        return 1;
    }

    // Mandatory inputs
    beam_id = std::stoi(argv[1]);
    phasemask = argv[2];

    // Optional mode
    if (argc >= 4) {
        observing_mode = argv[3];
        if (observing_mode != "bright" && observing_mode != "faint") {
            std::cerr << "[ERROR] Invalid observing mode. Must be 'bright' or 'faint'." << std::endl;
            return 1;
        }
    }

    // Optional user configuration file
    if (argc == 5) {
        user_config_filename = argv[4];
        user_provided_config = true;
    }

    std::string filename;
    if (user_provided_config) {
        filename = user_config_filename;
        std::cout << "[INFO] Using user-provided config file: " << filename << std::endl;
    } else {
        try {
            filename = find_latest_config_file(beam_id, observing_mode); // <--- updated find_latest_config_file
            std::cout << "[INFO] Using latest config for mode '" << observing_mode << "': " << filename << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error finding latest config file: " << e.what() << std::endl;
            return 1;
        }
    }

    // Load and parse TOML config
    try {
        config = toml::parse_file(filename);
        std::cout << "Loaded configuration for beam " << beam_id << " from " << filename << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing file " << filename << ": " << e.what() << std::endl;
        return 1;
    }

    // Now set up rtc_config
    std::string beamKey = "beam" + std::to_string(beam_id);
    try {
        rtc_config = readBDRConfig(config, beamKey, phasemask);
        rtc_config.initDerivedParameters();
        std::cout << "[INFO] RTC configuration initialized for beam " << beam_id << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing RTC config: " << e.what() << std::endl;
        return 1;
    }

    // Open shared memory
    if (!rtc_config.state.simulation_mode) {
        std::cout << "[INFO] Opening real SHM (not simulation)..." << std::endl;

        ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id)).c_str());

        std::string name = "dm" + std::to_string(beam_id) + "disp02";
        std::string name0 = "dm" + std::to_string(beam_id);

        if (ImageStreamIO_openIm(&dm_rtc, name.c_str()) != IMAGESTREAMIO_SUCCESS) {
            std::cerr << "[ERROR] Failed to open DM SHM: " << name << std::endl;
            return 1;
        }
        if (ImageStreamIO_openIm(&dm_rtc0, name0.c_str()) != IMAGESTREAMIO_SUCCESS) {
            std::cerr << "[ERROR] Failed to open DM master SHM: " << name0 << std::endl;
            return 1;
        }

    } else {
        std::cerr << "[ERROR] Simulation mode not implemented yet." << std::endl;
        throw std::runtime_error("Simulation mode not implemented");
    }

    // Start RTC threads
    std::thread rtc_thread(rtc);
    std::thread telemetry_thread(telemetry);

    std::cout << "[INFO] RTC and telemetry threads started." << std::endl;

    // Start commander server
    commander::Server s(argc, argv);
    s.run();

    // Clean shutdown
    servo_mode = SERVO_STOP;
    rtc_thread.join();
    telemetry_thread.join();

    std::cout << "DONE" << std::endl;
    return 0;
}

// before we introduced faint mode 
// // configure like ./baldr 1 H3 
// int main(int argc, char* argv[]) {
//     // We expect arguments: <beam_id> <phaseMask>
//     if (argc != 3) {
//         std::cerr << "Usage: " << argv[0] << " <beam_id1> <phaseMask1>" << std::endl;
//         return 1;
//     }
//     // if (argc != 3 && argc != 4) {
//     //     std::cerr << "Usage: " << argv[0] << " <beam_id1> <phaseMask1> [optional loop_time (us)]" << std::endl;
//     //     return 1;
//     // }

//     beam_id = std::stoi(argv[1]);
//     phasemask = argv[2];


//     // if (argc == 4) {
//     //     loop_time = std::stof(argv[3]);
//     //     loop_time_override = true;
//     //     if (loop_time <= 0.0f) {
//     //         std::cerr << "[ERROR] Invalid loop_time specified. Must be > 0." << std::endl;
//     //         return 1;
//     //     }
//     //     std::cout << "[INFO] User override loop_time = " << loop_time << " seconds (" << 1.0f / loop_time << " Hz)" << std::endl;
//     // }


//     // Compute the configuration filename and parse the TOML file.
//     //std::string filename = "/usr/local/etc/baldr/baldr_config_" + std::to_string(beam_id) + ".toml";

//     std::string filename;
//     try {
//         filename = find_latest_config_file(beam_id);
//         std::cout << "Using latest config: " << filename << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error finding latest config file: " << e.what() << std::endl;
//         exit(1);
//     }

//     try {
//         config = toml::parse_file(filename);
//         std::cout << "Loaded configuration for beam " << beam_id << " from " << filename << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error parsing file " << filename << ": " << e.what() << std::endl;
//         return 1;
//     }

//     //to create a global list of bdr_rtc_config structs:
//     std::string beamKey = "beam" + std::to_string(beam_id);
//     try {
//         rtc_config = readBDRConfig(config, beamKey, phasemask);
//         std::cout << "after read in baldr main rtc.ctrl_LO_config.kp.size() = " << rtc_config.ctrl_LO_config.kp.size() << std::endl;
//         rtc_config.initDerivedParameters();
//         std::cout << "finished init of run_time parameters" << std::endl;
//         std::cout << "after initDerived parameters in baldr main rtc.ctrl_LO_config.kp.size() = " << rtc_config.ctrl_LO_config.kp.size() << std::endl;

//         std::cout << "Initialized configuration for beam " << beam_id << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error initializing configuration for beam " << beam_id << ": " << e.what() << std::endl;
//         return 1;
//     }

//     // if user input for loop time then  - we should 
//     // if (!loop_time_override) {
//     //     loop_time = 1000000.0f / rtc_config.fps; // us seconds
//     //     std::cout << "[INFO] Default loop_time set from RTC config FPS: loop_time = " 
//     //               << loop_time << " seconds (" << rtc_config.fps << " Hz)" << std::endl;
//     // }


//     // The C-red image for the baldr subarray is /dev/shm/baldrN.im.shm, 
//     // where N is the beam number.
//     // 
//     //     ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id) ).c_str());
        
//     //     // Open the DM image. It is e.g. /dev/shm/dm2disp02.im.shm for beam 2.
//     //     std::string dm_filename = "dm" + std::to_string(beam_id) + "disp02" ;
//     //     ImageStreamIO_openIm(&dm_rtc, dm_filename.c_str());
//     // 
//     if (!rtc_config.state.simulation_mode) {
//         std::cout << "SHM configuration not in simulation mode" << std::endl;
//         // Real mode: open real images from the shared memory.
//         ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id)).c_str());

//         // Form the shared-memory name, e.g. "dm2disp02"
//         std::string name = "dm" + std::to_string(beam_id) + "disp02";
//         std::string name0 = "dm" + std::to_string(beam_id);
//         std::cout << "[INFO] Opening DM SHM \"" << name << "\"\n";
        
//         if (ImageStreamIO_openIm(&dm_rtc, name.c_str()) != IMAGESTREAMIO_SUCCESS) {
//             std::cerr << "[ERROR] Failed to open DM SHM \"" << name << "\"\n";
//             return 1;
//         }
//         if (ImageStreamIO_openIm(&dm_rtc0, name0.c_str()) != IMAGESTREAMIO_SUCCESS) {
//             std::cerr << "[ERROR] Failed to open DM SHM \"" << name0 << "\"\n";
//             return 1;
//         }
//         // BCB        
//         // // Open the DM image. (For beam 2, for example, this may be: "/dev/shm/dm2disp02.im.shm")
//         // std::string dm_filename = "dm" + std::to_string(beam_id) + "disp02";
//         // ImageStreamIO_openIm(&dm_rtc, dm_filename.c_str());

//         // // master process for DM 
//         // std::string name0 = "dm" + std::to_string(beam_id);
//         // ImageStreamIO_openIm(&dm_rtc0, name0.c_str());

//         // //add.
//         // ImageStreamIO_semflush(&dm_rtc0, /*index=*/-1);

//     } else {
//         // Simulation mode: if not yet implemented, raise an error.
//         std::cerr << "Simulation mode not implemented. Aborting." << std::endl;
//         throw std::runtime_error("Simulation mode not implemented");
        
//         // Alternatively, if you prefer to use dummy simulation images,
//         // you might do something like:
//         // ImageStreamIO_openIm(&subarray, "simulated_baldr.im.shm");
//         // ImageStreamIO_openIm(&dm_rtc, "simulated_dm.im.shm");
//     }
//     // Start the main RTC and telemetry threads. 


//     std::thread rtc_thread(rtc);
//     std::thread telemetry_thread(telemetry);

//     std::cout << "after rtc thread started in  main rtc.ctrl_LO_config.kp.size() = " << rtc_config.ctrl_LO_config.kp.size() << std::endl;
//     // Initialize the commander server and run it
//     commander::Server s(argc, argv);
//     s.run();

//     // Join the  thread
//     servo_mode = SERVO_STOP;
//     rtc_thread.join();
//     telemetry_thread.join();

//     // Continue with your RTC main loop using rtc_config_list...
//     std::cout << "DONE" << std::endl;
    
//     return 0;
// }

