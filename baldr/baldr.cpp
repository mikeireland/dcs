#define TOML_IMPLEMENTATION
#include "baldr.h"
#include "burst_window.h"
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


#include <unordered_map>
#include <functional>
#include <type_traits>

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

//// uncomment and build July 2025 AIV
std::atomic<int> global_boxcar{1};  // for possible weighted averaging of signals in rtc

IMAGE subarray; // The C-red subarray
IMAGE dm_rtc; // The DM subarray
IMAGE dm_rtc0; // The DM subarray master to post semaphores to


// foprward dec.
void pauseRTC();
void resumeRTC();

// Initialize global ZMQ variables
zmq::context_t cam_zmq_context(1);
zmq::socket_t cam_zmq_socket(cam_zmq_context, zmq::socket_type::req);
std::string cam_host_str = "tcp://192.168.100.2:6667";
bool cam_zmq_initialized = false;

// Initialize global ZMQ variables for MultiDeviceServer
zmq::context_t mds_zmq_context(1);
zmq::socket_t mds_zmq_socket(mds_zmq_context, zmq::socket_type::req);
std::string mds_host_str = "tcp://192.168.100.2:5555";
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


// Configure burst window (used for intensity slope estime) from current camera settings
//burst update here
void configure_burstWindow_from_camera(double phase = 0.0) {
    std::lock_guard<std::mutex> lock(rtc_mutex);

    try {
        // Query number of reads per reset and convert to integer
        std::string nbread_str = extract_value(send_cam_cmd("nbreadworeset"));
        int nbread = std::stoi(nbread_str);

        // Query current FPS
        float fps = get_float_cam_param("fps raw");

        // Sanity check
        if (nbread <= 0 || fps <= 0.0) {
            throw std::runtime_error("Invalid nbread or fps from camera");
        }

        // Configure the burst window
        rtc_config.burst.configure(nbread, fps, phase);


    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to configure burst window from camera: " << e.what() << std::endl;
    }
}
//commander compatible wrapper 
void cmd_configure_burst(json args) {
    double phase = 0.0;
    if (args.is_array() && !args.empty()) {
        phase = args.at(0).get<double>();
    }

    try {
        configure_burstWindow_from_camera(phase);
;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] failed to configure burst window " << e.what() << std::endl;
    }
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
            try{ 
                rtc.state.DM_flat = ctrl_tbl["DM_flat"] ? ctrl_tbl["DM_flat"].value_or(std::string("")) : "";
                rtc.state.signal_space  =  ctrl_tbl["signal_space"] ? ctrl_tbl["signal_space"].value_or(std::string("")) : ""; 
                rtc.state.LO = ctrl_tbl["LO"] ? ctrl_tbl["LO"].value_or(0) : 0;
                rtc.state.controller_type = ctrl_tbl["controller_type"] ? ctrl_tbl["controller_type"].value_or(std::string("")) : "";
                rtc.state.inverse_method_LO = ctrl_tbl["inverse_method_LO"] ? ctrl_tbl["inverse_method_LO"].value_or(std::string("")) : "";
                rtc.state.inverse_method_HO = ctrl_tbl["inverse_method_HO"] ? ctrl_tbl["inverse_method_HO"].value_or(std::string("")) : "";
                rtc.state.auto_close = ctrl_tbl["auto_close"] ? ctrl_tbl["auto_close"].value_or(int(0)) : 0;
                rtc.state.auto_open = ctrl_tbl["auto_open"] ? ctrl_tbl["auto_open"].value_or(int(1)): 1;
                rtc.state.auto_tune = ctrl_tbl["auto_tune"] ? ctrl_tbl["auto_tuen"].value_or(int(0)) : 0;
                rtc.state.simulation_mode = 1 ; 
            }catch (const std::exception& ex) {
                std::cerr << "Error with state read-in: " << ex.what() << std::endl;
                std::exit(1);
            }
            // pix
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
                //rtc.matrices.I2M = convertTomlArrayToEigenMatrix(*ctrl_tbl["I2M"].as_array(),rtc.matrices.I2M,"I2M");
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

            try{
                // limits float 
                rtc.limits.close_on_strehl_limit = ctrl_tbl["close_on_strehl_limit"].value_or(0.0);
                rtc.limits.open_on_strehl_limit = ctrl_tbl["open_on_strehl_limit"].value_or(0.0);
                rtc.limits.open_on_flux_limit = ctrl_tbl["open_on_flux_limit"].value_or(0.0);
                rtc.limits.open_on_dm_limit = ctrl_tbl["open_on_dm_limit"].value_or(0.0);
                rtc.limits.LO_offload_limit = ctrl_tbl["LO_offload_limit"].value_or(0.0);
            }catch(const std::exception& ex) {
                std::cerr << "Error with limits readin " << ex.what() << std::endl;
                std::exit(1);
            }
            // cam config when building interaction matrix 
            try{
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
            catch(const std::exception& ex) {
                std::cerr << "Error with camera configuration readin" << ex.what() << std::endl;
                std::exit(1);
            }
        
    }
    }

    std::cout << "here here" << std::endl;

    try{    
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


    }catch (const std::exception& ex) {
        std::cerr << "Error initializing telemetry or controller: " << ex.what() << std::endl;
        std::exit(1);
    }

    try{
        // Validate the master configuration.
        rtc.validate();
    }catch(const std::exception& ex) {
        std::cerr << "Error validating RTC configuration: " << ex.what() << std::endl;
        std::exit(1);
    }

    return rtc;
}


//uncomment and build July 2025 AIV
void change_boxcar(int cmd) {
    try {
        int new_val = cmd; //std::stoi(cmd);
        if (new_val > 0 && new_val < 1000) {
            global_boxcar.store(new_val); //atomic threadsafe
            std::cout << "Boxcar window updated to " << new_val << std::endl;
        } else {
            std::cerr << "Boxcar value out of allowed range (1–999)" << std::endl;
        }
    } catch (...) {
        std::cerr << "Invalid boxcar input: " << cmd << std::endl;
    }
}

// test in TTR115.0035 (not tested yet)
// JSON-returning version compatible with Commander
nlohmann::json poll_telem_vector(std::string name) {
    static const std::unordered_map<std::string,
        boost::circular_buffer<Eigen::VectorXd> bdr_telem::*> telemetry_map = {
        {"img",    &bdr_telem::img},
        {"img_dm", &bdr_telem::img_dm},
        {"signal", &bdr_telem::signal},
        {"e_LO",   &bdr_telem::e_LO},
        {"u_LO",   &bdr_telem::u_LO},
        {"e_HO",   &bdr_telem::e_HO},
        {"u_HO",   &bdr_telem::u_HO},
        {"c_LO",   &bdr_telem::c_LO},
        {"c_HO",   &bdr_telem::c_HO}
    };

    nlohmann::json j;

    auto it = telemetry_map.find(name);
    if (it == telemetry_map.end()) {
        // consistent, explicit error payload
        return nlohmann::json{{"ok", false}, {"error", "unknown field"}, {"field", name}};
    }

    std::lock_guard<std::mutex> lk(telemetry_mutex);

    const auto& buffer = rtc_config.telem.*(it->second);
    const auto n = buffer.size();
    if (n < 2) {
        // “no data yet” -> null data with ok=false
        return nlohmann::json{{"ok", false}, {"error", "insufficient data"}, {"size", n}};
    }

    const Eigen::VectorXd& v = buffer[n - 2];
    // Convert Eigen -> std::vector<double> -> JSON array
    std::vector<double> out(v.data(), v.data() + v.size());

    j = nlohmann::json{
        {"ok",   true},
        {"name", name},
        {"size", v.size()},
        {"data", out}
    };
    return j;
}


// // test in TTR115.0035 (not tested yet)
// JSON-returning scalar poll (Commander-friendly)
nlohmann::json poll_telem_scalar(std::string name) {
    // Only true scalars here. (LO/HO states are ints—expose via a separate command if needed.)
    static const std::unordered_map<std::string,
        boost::circular_buffer<double> bdr_telem::*> scalar_map = {
        {"rmse_est", &bdr_telem::rmse_est},
        {"snr",      &bdr_telem::snr},
        // add more scalar fields here if you have them
    };

    auto it = scalar_map.find(name);
    if (it == scalar_map.end()) {
        return nlohmann::json{{"ok", false}, {"error", "unknown field"}, {"field", name}};
    }

    std::lock_guard<std::mutex> lk(telemetry_mutex);

    const auto& buffer = rtc_config.telem.*(it->second);
    const auto n = buffer.size();
    if (n < 2) {
        return nlohmann::json{{"ok", false}, {"error", "insufficient data"}, {"size", n}};
    }

    const double val = buffer[n - 2];
    return nlohmann::json{
        {"ok",   true},
        {"name", name},
        {"data", val}
    };
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


///// untested version 2 of getter and setters! 
struct FieldHandle {
    std::string type;  // for introspection ("double","int","string","vector","matrix",…)
    std::function<nlohmann::json()> get;
    std::function<bool(const nlohmann::json&)> set; // nullptr => read-only
};

// ---- Eigen <-> JSON helpers ----
template<typename EigenVec>
static nlohmann::json eigen_vector_to_json(const EigenVec& v) {
    using S = typename EigenVec::Scalar;
    if constexpr (std::is_integral_v<S>) {
        std::vector<long long> out(v.size());
        for (Eigen::Index i = 0; i < v.size(); ++i) out[i] = static_cast<long long>(v(i));
        return out;
    } else {
        std::vector<double> out(v.size());
        for (Eigen::Index i = 0; i < v.size(); ++i) out[i] = static_cast<double>(v(i));
        return out;
    }
}

template<typename EigenMat>
static nlohmann::json eigen_matrix_to_json(const EigenMat& M) {
    using S = typename EigenMat::Scalar;
    nlohmann::json rows = nlohmann::json::array();
    for (Eigen::Index r = 0; r < M.rows(); ++r) {
        if constexpr (std::is_integral_v<S>) {
            std::vector<long long> row(M.cols());
            for (Eigen::Index c = 0; c < M.cols(); ++c) row[c] = static_cast<long long>(M(r,c));
            rows.push_back(row);
        } else {
            std::vector<double> row(M.cols());
            for (Eigen::Index c = 0; c < M.cols(); ++c) row[c] = static_cast<double>(M(r,c));
            rows.push_back(row);
        }
    }
    return rows;
}

template<typename EigenVec>
static bool json_to_eigen_vector(const nlohmann::json& j, EigenVec& out) {
    if (!j.is_array()) return false;
    using S = typename EigenVec::Scalar;
    const auto n = j.size();
    out.resize(static_cast<Eigen::Index>(n));
    try {
        for (size_t i = 0; i < n; ++i) out(static_cast<Eigen::Index>(i)) = static_cast<S>(j.at(i).get<double>());
        return true;
    } catch (...) { return false; }
}

template<typename EigenMat>
static bool json_to_eigen_matrix(const nlohmann::json& j, EigenMat& out) {
    if (!j.is_array() || j.empty() || !j.front().is_array()) return false;
    const auto rows = j.size();
    const auto cols = j.front().size();
    for (const auto& row : j) if (!row.is_array() || row.size() != cols) return false;
    out.resize(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
    try {
        for (size_t r = 0; r < rows; ++r)
            for (size_t c = 0; c < cols; ++c)
                out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
                    static_cast<typename EigenMat::Scalar>(j[r][c].get<double>());
        return true;
    } catch (...) { return false; }
}

// ---- Getter-only factories ----
template<typename T>
static FieldHandle make_scalar_field_getter(T bdr_rtc_config::* ptr, const char* type_name) {
    return FieldHandle{
        type_name,
        [ptr]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return rtc_config.*ptr; },
        nullptr
    };
}
template<typename Sub, typename T>
static FieldHandle make_nested_scalar_getter(Sub bdr_rtc_config::* sub, T Sub::* mem, const char* type_name) {
    return FieldHandle{
        type_name,
        [sub, mem]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return (rtc_config.*sub).*mem; },
        nullptr
    };
}
template<typename T>
static FieldHandle make_vector_field_getter(std::vector<T> bdr_rtc_config::* ptr, const char* type_name) {
    return FieldHandle{
        type_name,
        [ptr]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return rtc_config.*ptr; },
        nullptr
    };
}
template<typename Sub, typename EigenVec>
static FieldHandle make_nested_eigen_vector_getter(Sub bdr_rtc_config::* sub, EigenVec Sub::* mem, const char* tn="vector") {
    return FieldHandle{
        tn,
        [sub, mem]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return eigen_vector_to_json((rtc_config.*sub).*mem); },
        nullptr
    };
}
template<typename Sub, typename EigenMat>
static FieldHandle make_nested_eigen_matrix_getter(Sub bdr_rtc_config::* sub, EigenMat Sub::* mem, const char* tn="matrix") {
    return FieldHandle{
        tn,
        [sub, mem]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return eigen_matrix_to_json((rtc_config.*sub).*mem); },
        nullptr
    };
}
template<typename EigenVec>
static FieldHandle make_eigen_vector_field_getter(EigenVec bdr_rtc_config::* mem, const char* tn="vector") {
    return FieldHandle{
        tn,
        [mem]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return eigen_vector_to_json(rtc_config.*mem); },
        nullptr
    };
}
template<typename EigenMat>
static FieldHandle make_eigen_matrix_field_getter(EigenMat bdr_rtc_config::* mem, const char* tn="matrix") {
    return FieldHandle{
        tn,
        [mem]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return eigen_matrix_to_json(rtc_config.*mem); },
        nullptr
    };
}

// ---- Read/Write factories (use these for fields you want to mutate) ----
template<typename T>
static FieldHandle make_scalar_field_rw(T bdr_rtc_config::* ptr, const char* type_name) {
    return FieldHandle{
        type_name,
        [ptr]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return rtc_config.*ptr; },
        [ptr](const nlohmann::json& j) -> bool {
            try { T v = j.get<T>(); std::lock_guard<std::mutex> lk(rtc_mutex); rtc_config.*ptr = v; return true; }
            catch (...) { return false; }
        }
    };
}
template<typename Sub, typename T>
static FieldHandle make_nested_scalar_rw(Sub bdr_rtc_config::* sub, T Sub::* mem, const char* type_name) {
    return FieldHandle{
        type_name,
        [sub, mem]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return (rtc_config.*sub).*mem; },
        [sub, mem](const nlohmann::json& j) -> bool {
            try { T v = j.get<T>(); std::lock_guard<std::mutex> lk(rtc_mutex); (rtc_config.*sub).*mem = v; return true; }
            catch (...) { return false; }
        }
    };
}
template<typename T>
static FieldHandle make_vector_rw(std::vector<T> bdr_rtc_config::* ptr, const char* type_name) {
    return FieldHandle{
        type_name,
        [ptr]() -> nlohmann::json { std::lock_guard<std::mutex> lk(rtc_mutex); return rtc_config.*ptr; },
        [ptr](const nlohmann::json& j) -> bool {
            if (!j.is_array()) return false;
            try {
                std::vector<T> v = j.get<std::vector<T>>();
                std::lock_guard<std::mutex> lk(rtc_mutex);
                rtc_config.*ptr = std::move(v);
                return true;
            } catch (...) { return false; }
        }
    };
}
template<typename Sub, typename EigenVec>
static FieldHandle make_nested_eigen_vector_rw(Sub bdr_rtc_config::* sub, EigenVec Sub::* mem, const char* tn="vector<double>") {
    return FieldHandle{
        tn,
        [sub, mem]() -> nlohmann::json {
            std::lock_guard<std::mutex> lk(rtc_mutex);
            return eigen_vector_to_json((rtc_config.*sub).*mem);
        },
        [sub, mem](const nlohmann::json& j) -> bool {
            typename std::remove_reference_t<decltype((rtc_config.*sub).*mem)> tmp;
            if (!json_to_eigen_vector(j, tmp)) return false;
            std::lock_guard<std::mutex> lk(rtc_mutex);
            (rtc_config.*sub).*mem = std::move(tmp);
            return true;
        }
    };
}
template<typename Sub, typename EigenMat>
static FieldHandle make_nested_eigen_matrix_rw(Sub bdr_rtc_config::* sub, EigenMat Sub::* mem, const char* tn="matrix<double>") {
    return FieldHandle{
        tn,
        [sub, mem]() -> nlohmann::json {
            std::lock_guard<std::mutex> lk(rtc_mutex);
            return eigen_matrix_to_json((rtc_config.*sub).*mem);
        },
        [sub, mem](const nlohmann::json& j) -> bool {
            typename std::remove_reference_t<decltype((rtc_config.*sub).*mem)> tmp;
            if (!json_to_eigen_matrix(j, tmp)) return false;
            std::lock_guard<std::mutex> lk(rtc_mutex);
            (rtc_config.*sub).*mem = std::move(tmp);
            return true;
        }
    };
}

// --------- Registry of exposed fields on rtc_config ---------
// Telemetry ring buffers (rtc_config.telem.*) are intentionally omitted; you already access them via poll_telem_*.

static const std::unordered_map<std::string, FieldHandle> RTC_FIELDS = {
    // ===== state (RO) =====
    {"state.DM_flat",            make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::DM_flat, "string")},
    {"state.signal_space",       make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::signal_space, "string")},
    {"state.LO",                 make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::LO, "int")},
    {"state.controller_type",    make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::controller_type, "string")},
    {"state.inverse_method_LO",  make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::inverse_method_LO, "string")},
    {"state.inverse_method_HO",  make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::inverse_method_HO, "string")},
    {"state.phasemask",          make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::phasemask, "string")},
    {"state.auto_close",         make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::auto_close, "int")},
    {"state.auto_open",          make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::auto_open, "int")},
    {"state.auto_tune",          make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::auto_tune, "int")},
    {"state.take_telemetry",     make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::take_telemetry, "int")},
    {"state.simulation_mode",    make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::simulation_mode, "int")},

    // ===== reduction (RO) =====
    {"reduction.bias",           make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::bias)},
    {"reduction.bias_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::bias_dm)},
    {"reduction.dark",           make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::dark)},
    {"reduction.dark_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::dark_dm)},

    // ===== pixels (RO) =====
    {"pixels.crop_pixels",       make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::crop_pixels, "vector<int16>")},
    {"pixels.bad_pixels",        make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::bad_pixels, "vector<int32>")},
    {"pixels.pupil_pixels",      make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::pupil_pixels, "vector<int32>")},
    {"pixels.interior_pixels",   make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::interior_pixels, "vector<int32>")},
    {"pixels.secondary_pixels",  make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::secondary_pixels, "vector<int32>")},
    {"pixels.exterior_pixels",   make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::exterior_pixels, "vector<int32>")},

    // ===== reference_pupils (RO) =====
    {"reference_pupils.I0",           make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::I0)},
    {"reference_pupils.N0",           make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::N0)},
    {"reference_pupils.norm_pupil",   make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::norm_pupil)},
    {"reference_pupils.norm_pupil_dm",make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::norm_pupil_dm)},
    {"reference_pupils.I0_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::I0_dm)},

    // ===== matrices (RO) =====
    {"matrices.I2A",        make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2A)},
    {"matrices.I2M",        make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2M)},
    {"matrices.I2M_LO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2M_LO)},
    {"matrices.I2M_HO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2M_HO)},
    {"matrices.M2C",        make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::M2C)},
    {"matrices.M2C_LO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::M2C_LO)},
    {"matrices.M2C_HO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::M2C_HO)},
    {"matrices.I2rms_sec",  make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2rms_sec)},
    {"matrices.I2rms_ext",  make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2rms_ext)},
    {"matrices.szm",        make_nested_scalar_getter(&bdr_rtc_config::matrices, &bdr_matricies::szm, "int")},
    {"matrices.sza",        make_nested_scalar_getter(&bdr_rtc_config::matrices, &bdr_matricies::sza, "int")},
    {"matrices.szp",        make_nested_scalar_getter(&bdr_rtc_config::matrices, &bdr_matricies::szp, "int")},

    // ===== controller configs (RW ) =====
    {"ctrl_LO_config.kp",           make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::kp)},
    {"ctrl_LO_config.ki",           make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::ki)},
    {"ctrl_LO_config.kd",           make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::kd)},
    {"ctrl_LO_config.lower_limits", make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::lower_limits)},
    {"ctrl_LO_config.upper_limits", make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::upper_limits)},
    {"ctrl_LO_config.set_point",    make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::set_point)},

    {"ctrl_HO_config.kp",           make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::kp)},
    {"ctrl_HO_config.ki",           make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::ki)},
    {"ctrl_HO_config.kd",           make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::kd)},
    {"ctrl_HO_config.lower_limits", make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::lower_limits)},
    {"ctrl_HO_config.upper_limits", make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::upper_limits)},
    {"ctrl_HO_config.set_point",    make_nested_eigen_vector_rw(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::set_point)},

    // ===== limits (RW ) =====
    {"limits.close_on_strehl_limit", make_nested_scalar_rw(&bdr_rtc_config::limits, &bdr_limits::close_on_strehl_limit, "float")},
    {"limits.open_on_strehl_limit",  make_nested_scalar_rw(&bdr_rtc_config::limits, &bdr_limits::open_on_strehl_limit, "float")},
    {"limits.open_on_flux_limit",    make_nested_scalar_rw(&bdr_rtc_config::limits, &bdr_limits::open_on_flux_limit, "float")},
    {"limits.open_on_dm_limit",      make_nested_scalar_rw(&bdr_rtc_config::limits, &bdr_limits::open_on_dm_limit, "float")},
    {"limits.LO_offload_limit",      make_nested_scalar_rw(&bdr_rtc_config::limits, &bdr_limits::LO_offload_limit, "float")},

    // ===== cam (RO; strings control acquisition pipeline externally) =====
    {"cam.fps",               make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::fps, "string")},
    {"cam.gain",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::gain, "string")},
    {"cam.testpattern",       make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::testpattern, "string")},
    {"cam.bias",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::bias, "string")},
    {"cam.flat",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::flat, "string")},
    {"cam.imagetags",         make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::imagetags, "string")},
    {"cam.led",               make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::led, "string")},
    {"cam.events",            make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::events, "string")},
    {"cam.extsynchro",        make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::extsynchro, "string")},
    {"cam.rawimages",         make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::rawimages, "string")},
    {"cam.cooling",           make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cooling, "string")},
    {"cam.mode",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::mode, "string")},
    {"cam.resetwidth",        make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::resetwidth, "string")},
    {"cam.nbreadworeset",     make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::nbreadworeset, "string")},
    {"cam.cropping",          make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cropping, "string")},
    {"cam.cropping_columns",  make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cropping_columns, "string")},
    {"cam.cropping_rows",     make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cropping_rows, "string")},
    {"cam.aduoffset",         make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::aduoffset, "string")},

    // ===== filters (RO) =====
    {"filters.bad_pixel_mask",     make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::bad_pixel_mask, "vector<uint8>")},
    {"filters.bad_pixel_mask_dm",  make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::bad_pixel_mask_dm, "vector<float>")},
    {"filters.pupil",              make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::pupil, "vector<uint8>")},
    {"filters.pupil_dm",           make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::pupil_dm, "vector<float>")},
    {"filters.secondary",          make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::secondary, "vector<uint8>")},
    {"filters.secondary_dm",       make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::secondary_dm, "vector<float>")},
    {"filters.exterior",           make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::exterior, "vector<uint8>")},
    {"filters.exterior_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::exterior_dm, "vector<float>")},
    {"filters.inner_pupil_filt",   make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::inner_pupil_filt, "vector<uint8>")},
    {"filters.inner_pupil_filt_dm",make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::inner_pupil_filt_dm, "vector<float>")},

    // ===== derived/runtime (top-level) =====
    {"zeroCmd",              make_eigen_vector_field_getter(&bdr_rtc_config::zeroCmd, "vector<double>")}, // RO by default
    {"fps",                  make_scalar_field_rw(&bdr_rtc_config::fps,   "double")},  // RW
    {"gain",                 make_scalar_field_rw(&bdr_rtc_config::gain,  "double")},  // RW
    {"scale",                make_scalar_field_rw(&bdr_rtc_config::scale, "double")},  // RW
    {"I2M_LO_runtime",       make_eigen_matrix_field_getter(&bdr_rtc_config::I2M_LO_runtime, "matrix<double>")},
    {"I2M_HO_runtime",       make_eigen_matrix_field_getter(&bdr_rtc_config::I2M_HO_runtime, "matrix<double>")},
    {"N0_dm_runtime",        make_eigen_vector_field_getter(&bdr_rtc_config::N0_dm_runtime, "vector<double>")},
    {"I0_dm_runtime",        make_eigen_vector_field_getter(&bdr_rtc_config::I0_dm_runtime, "vector<double>")},
    {"dark_dm_runtime",      make_eigen_vector_field_getter(&bdr_rtc_config::dark_dm_runtime, "vector<double>")},
    {"sec_idx",              make_scalar_field_getter(&bdr_rtc_config::sec_idx, "int")},
    {"m_s_runtime",          make_scalar_field_getter(&bdr_rtc_config::m_s_runtime, "double")},
    {"b_s_runtime",          make_scalar_field_getter(&bdr_rtc_config::b_s_runtime, "double")},

    // ===== runtime controllers (RO unless you explicitly want to poke internals) =====
    {"ctrl_LO.ctrl_type",    make_nested_scalar_getter(&bdr_rtc_config::ctrl_LO, &PIDController::ctrl_type, "string")},
    {"ctrl_LO.kp",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::kp)},
    {"ctrl_LO.ki",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::ki)},
    {"ctrl_LO.kd",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::kd)},
    {"ctrl_LO.lower_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::lower_limits)},
    {"ctrl_LO.upper_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::upper_limits)},
    {"ctrl_LO.set_point",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::set_point)},
    {"ctrl_LO.output",       make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::output)},
    {"ctrl_LO.integrals",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::integrals)},
    {"ctrl_LO.prev_errors",  make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::prev_errors)},

    {"ctrl_HO.ctrl_type",    make_nested_scalar_getter(&bdr_rtc_config::ctrl_HO, &PIDController::ctrl_type, "string")},
    {"ctrl_HO.kp",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::kp)},
    {"ctrl_HO.ki",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::ki)},
    {"ctrl_HO.kd",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::kd)},
    {"ctrl_HO.lower_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::lower_limits)},
    {"ctrl_HO.upper_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::upper_limits)},
    {"ctrl_HO.set_point",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::set_point)},
    {"ctrl_HO.output",       make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::output)},
    {"ctrl_HO.integrals",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::integrals)},
    {"ctrl_HO.prev_errors",  make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::prev_errors)}
};

nlohmann::json list_rtc_fields() {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& kv : RTC_FIELDS) {
        arr.push_back({{"field", kv.first}, {"type", kv.second.type}});
    }
    return {{"ok", true}, {"fields", arr}};
}

nlohmann::json get_rtc_field(std::string path) {
    auto it = RTC_FIELDS.find(path);
    if (it == RTC_FIELDS.end()) {
        nlohmann::json keys = nlohmann::json::array();
        for (const auto& kv : RTC_FIELDS) keys.push_back(kv.first);
        return {{"ok", false}, {"error", "unknown field"}, {"field", path}, {"available", keys}};
    }
    const auto& fh = it->second;
    return {{"ok", true}, {"field", path}, {"type", fh.type}, {"value", fh.get()}};
}

nlohmann::json set_rtc_field(std::string path, nlohmann::json value) {
    auto it = RTC_FIELDS.find(path);
    if (it == RTC_FIELDS.end())
        return {{"ok", false}, {"error", "unknown field"}, {"field", path}};
    const auto& fh = it->second;
    if (!fh.set)
        return {{"ok", false}, {"error", "read-only field"}, {"field", path}};
    // Optional: per-field validation here (range/size checks, etc.)
    const bool ok = fh.set(value);
    if (!ok)
        return {{"ok", false}, {"error", "bad value/type"}, {"field", path}, {"expected", fh.type}};
    // Side-effects hook (recompute derived state) — add as needed, e.g.:
    // if (path.rfind("ctrl_", 0) == 0) { std::lock_guard<std::mutex> lk(rtc_mutex); /* rebuild controllers */ }
    return {{"ok", true}, {"field", path}, {"type", fh.type}, {"value", fh.get()}};
}



//// working version 1 
// //// rtc config file getter and setters ! 
// // // test in TTR115.0035 (not tested yet)

// // A handler for a single exposed field on rtc_config.
// // We only USE .get() now; .set() is reserved for later.
// struct FieldHandle {
//     std::string type;                             // "double", "int", "bool", "string", "vector<double>", ...
//     std::function<nlohmann::json()> get;          // returns current value as JSON
//     std::function<bool(const nlohmann::json&)> set;  // optional; unused for now
// };

// // --- Helper factories: GETTER ONLY now; set = nullptr for expansion later ---

// template<typename T>
// static FieldHandle make_scalar_field_getter(T bdr_rtc_config::* ptr, const char* type_name) {
//     return FieldHandle{
//         type_name,
//         // getter
//         [ptr]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return rtc_config.*ptr;               // nlohmann::json supports numbers/bools/strings directly
//         },
//         /*set*/ nullptr
//     };
// }

// template<typename Sub, typename T>
// static FieldHandle make_nested_scalar_getter(Sub bdr_rtc_config::* sub, T Sub::* mem, const char* type_name) {
//     return FieldHandle{
//         type_name,
//         [sub, mem]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return (rtc_config.*sub).*mem;
//         },
//         /*set*/ nullptr
//     };
// }

// template<typename T>
// static FieldHandle make_vector_field_getter(std::vector<T> bdr_rtc_config::* ptr, const char* type_name) {
//     return FieldHandle{
//         type_name,
//         [ptr]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return rtc_config.*ptr;               // std::vector<T> -> JSON array
//         },
//         /*set*/ nullptr
//     };
// }

// // Helper for nested Eigen::VectorXd -> JSON array
// template<typename Sub>
// static FieldHandle make_nested_eigenvec_getter(Sub bdr_rtc_config::* sub,
//                                                Eigen::VectorXd Sub::* mem,
//                                                const char* type_name = "vector<double>") {
//     return FieldHandle{
//         type_name,
//         [sub, mem]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             const Eigen::VectorXd& v = (rtc_config.*sub).*mem;
//             return std::vector<double>(v.data(), v.data() + v.size());
//         },
//         /*set*/ nullptr
//     };
// }

// // ---------- Generic Eigen -> JSON helpers (vectors & matrices) ----------
// template<typename EigenVec>
// static nlohmann::json eigen_vector_to_json(const EigenVec& v) {
//     using S = typename EigenVec::Scalar;
//     // Avoid char-like JSON by widening integral scalars
//     if constexpr (std::is_integral_v<S>) {
//         std::vector<long long> out(v.size());
//         for (Eigen::Index i = 0; i < v.size(); ++i) out[i] = static_cast<long long>(v(i));
//         return out;
//     } else {
//         std::vector<double> out(v.size());
//         for (Eigen::Index i = 0; i < v.size(); ++i) out[i] = static_cast<double>(v(i));
//         return out;
//     }
// }

// template<typename EigenMat>
// static nlohmann::json eigen_matrix_to_json(const EigenMat& M) {
//     using S = typename EigenMat::Scalar;
//     nlohmann::json rows = nlohmann::json::array();
//     for (Eigen::Index r = 0; r < M.rows(); ++r) {
//         if constexpr (std::is_integral_v<S>) {
//             std::vector<long long> row(M.cols());
//             for (Eigen::Index c = 0; c < M.cols(); ++c) row[c] = static_cast<long long>(M(r,c));
//             rows.push_back(row);
//         } else {
//             std::vector<double> row(M.cols());
//             for (Eigen::Index c = 0; c < M.cols(); ++c) row[c] = static_cast<double>(M(r,c));
//             rows.push_back(row);
//         }
//     }
//     return rows;
// }

// // ---------- FieldHandle factories for Eigen (nested and top-level) ----------
// template<typename Sub, typename EigenVec>
// static FieldHandle make_nested_eigen_vector_getter(Sub bdr_rtc_config::* sub,
//                                                    EigenVec Sub::* mem,
//                                                    const char* type_name = "vector") {
//     return FieldHandle{
//         type_name,
//         [sub, mem]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return eigen_vector_to_json((rtc_config.*sub).*mem);
//         },
//         /*set*/ nullptr
//     };
// }

// template<typename Sub, typename EigenMat>
// static FieldHandle make_nested_eigen_matrix_getter(Sub bdr_rtc_config::* sub,
//                                                    EigenMat Sub::* mem,
//                                                    const char* type_name = "matrix") {
//     return FieldHandle{
//         type_name,
//         [sub, mem]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return eigen_matrix_to_json((rtc_config.*sub).*mem);
//         },
//         /*set*/ nullptr
//     };
// }

// template<typename EigenVec>
// static FieldHandle make_eigen_vector_field_getter(EigenVec bdr_rtc_config::* mem,
//                                                   const char* type_name = "vector") {
//     return FieldHandle{
//         type_name,
//         [mem]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return eigen_vector_to_json(rtc_config.*mem);
//         },
//         /*set*/ nullptr
//     };
// }

// template<typename EigenMat>
// static FieldHandle make_eigen_matrix_field_getter(EigenMat bdr_rtc_config::* mem,
//                                                   const char* type_name = "matrix") {
//     return FieldHandle{
//         type_name,
//         [mem]() -> nlohmann::json {
//             std::lock_guard<std::mutex> lk(rtc_mutex);
//             return eigen_matrix_to_json(rtc_config.*mem);
//         },
//         /*set*/ nullptr
//     };
// }

// // // ---------------- Register ONLY the fields we want visible ----------------
// // // Start empty; add lines like the commented examples below when ready.
// // // NOTE: Update names to match your actual bdr_rtc_config definition.
// // static const std::unordered_map<std::string, FieldHandle> RTC_FIELDS = {
// //     // Example top-level scalars:
// //     // {"fps_override",      make_scalar_field_getter(&bdr_rtc_config::fps_override, "double")},
// //     // {"gain_override",     make_scalar_field_getter(&bdr_rtc_config::gain_override, "double")},
// //     // {"override_gain_fps", make_scalar_field_getter(&bdr_rtc_config::override_gain_fps, "bool")},

// //     // Example nested (replace 'CtrlType' and members with your real types):
// //     // {"ctrl_LO.kp", make_nested_scalar_getter(&bdr_rtc_config::ctrl_LO, &CtrlType::kp, "double")},
// //     { "ctrl_LO.ki",make_nested_eigenvec_getter(&bdr_rtc_config::ctrl_LO, &PIDController::ki) },
// //     // {"ctrl_HO.rho", make_nested_scalar_getter(&bdr_rtc_config::ctrl_HO, &LeakyIntegrator::rho, "double")},

// //     // Example vector:
// //     // {"modal_gains_LO", make_vector_field_getter(&bdr_rtc_config::modal_gains_LO, "vector<double>")},
// // };
// static const std::unordered_map<std::string, FieldHandle> RTC_FIELDS = {
//     // ----- state -----
//     {"state.DM_flat",            make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::DM_flat, "string")},
//     {"state.signal_space",       make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::signal_space, "string")},
//     {"state.LO",                 make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::LO, "int")},
//     {"state.controller_type",    make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::controller_type, "string")},
//     {"state.inverse_method_LO",  make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::inverse_method_LO, "string")},
//     {"state.inverse_method_HO",  make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::inverse_method_HO, "string")},
//     {"state.phasemask",          make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::phasemask, "string")},
//     {"state.auto_close",         make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::auto_close, "int")},
//     {"state.auto_open",          make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::auto_open, "int")},
//     {"state.auto_tune",          make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::auto_tune, "int")},
//     {"state.take_telemetry",     make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::take_telemetry, "int")},
//     {"state.simulation_mode",    make_nested_scalar_getter(&bdr_rtc_config::state, &bdr_state::simulation_mode, "int")},

//     // ----- reduction (Eigen::VectorXd) -----
//     {"reduction.bias",           make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::bias, "vector<double>")},
//     {"reduction.bias_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::bias_dm, "vector<double>")},
//     {"reduction.dark",           make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::dark, "vector<double>")},
//     {"reduction.dark_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::reduction, &bdr_reduction::dark_dm, "vector<double>")},

//     // ----- pixels (integer Eigen vectors) -----
//     {"pixels.crop_pixels",       make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::crop_pixels, "vector<int16>")},
//     {"pixels.bad_pixels",        make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::bad_pixels, "vector<int32>")},
//     {"pixels.pupil_pixels",      make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::pupil_pixels, "vector<int32>")},
//     {"pixels.interior_pixels",   make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::interior_pixels, "vector<int32>")},
//     {"pixels.secondary_pixels",  make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::secondary_pixels, "vector<int32>")},
//     {"pixels.exterior_pixels",   make_nested_eigen_vector_getter(&bdr_rtc_config::pixels, &bdr_pixels::exterior_pixels, "vector<int32>")},

//     // ----- reference_pupils (Eigen::VectorXd) -----
//     {"reference_pupils.I0",           make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::I0, "vector<double>")},
//     {"reference_pupils.N0",           make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::N0, "vector<double>")},
//     {"reference_pupils.norm_pupil",   make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::norm_pupil, "vector<double>")},
//     {"reference_pupils.norm_pupil_dm",make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::norm_pupil_dm, "vector<double>")},
//     {"reference_pupils.I0_dm",        make_nested_eigen_vector_getter(&bdr_rtc_config::reference_pupils, &bdr_refence_pupils::I0_dm, "vector<double>")},

//     // ----- matrices (Eigen::MatrixXd) + sizes -----
//     {"matrices.I2A",        make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2A, "matrix<double>")},
//     {"matrices.I2M",        make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2M, "matrix<double>")},
//     {"matrices.I2M_LO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2M_LO, "matrix<double>")},
//     {"matrices.I2M_HO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2M_HO, "matrix<double>")},
//     {"matrices.M2C",        make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::M2C, "matrix<double>")},
//     {"matrices.M2C_LO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::M2C_LO, "matrix<double>")},
//     {"matrices.M2C_HO",     make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::M2C_HO, "matrix<double>")},
//     {"matrices.I2rms_sec",  make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2rms_sec, "matrix<double>")},
//     {"matrices.I2rms_ext",  make_nested_eigen_matrix_getter(&bdr_rtc_config::matrices, &bdr_matricies::I2rms_ext, "matrix<double>")},
//     {"matrices.szm",        make_nested_scalar_getter(&bdr_rtc_config::matrices, &bdr_matricies::szm, "int")},
//     {"matrices.sza",        make_nested_scalar_getter(&bdr_rtc_config::matrices, &bdr_matricies::sza, "int")},
//     {"matrices.szp",        make_nested_scalar_getter(&bdr_rtc_config::matrices, &bdr_matricies::szp, "int")},

//     // ----- LO/HO controller configs (bdr_controller) -----
//     {"ctrl_LO_config.kp",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::kp, "vector<double>")},
//     {"ctrl_LO_config.ki",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::ki, "vector<double>")},
//     {"ctrl_LO_config.kd",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::kd, "vector<double>")},
//     {"ctrl_LO_config.lower_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::lower_limits, "vector<double>")},
//     {"ctrl_LO_config.upper_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::upper_limits, "vector<double>")},
//     {"ctrl_LO_config.set_point",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO_config, &bdr_controller::set_point, "vector<double>")},

//     {"ctrl_HO_config.kp",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::kp, "vector<double>")},
//     {"ctrl_HO_config.ki",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::ki, "vector<double>")},
//     {"ctrl_HO_config.kd",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::kd, "vector<double>")},
//     {"ctrl_HO_config.lower_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::lower_limits, "vector<double>")},
//     {"ctrl_HO_config.upper_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::upper_limits, "vector<double>")},
//     {"ctrl_HO_config.set_point",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO_config, &bdr_controller::set_point, "vector<double>")},

//     // ----- loop limits -----
//     {"limits.close_on_strehl_limit", make_nested_scalar_getter(&bdr_rtc_config::limits, &bdr_limits::close_on_strehl_limit, "float")},
//     {"limits.open_on_strehl_limit",  make_nested_scalar_getter(&bdr_rtc_config::limits, &bdr_limits::open_on_strehl_limit, "float")},
//     {"limits.open_on_flux_limit",    make_nested_scalar_getter(&bdr_rtc_config::limits, &bdr_limits::open_on_flux_limit, "float")},
//     {"limits.open_on_dm_limit",      make_nested_scalar_getter(&bdr_rtc_config::limits, &bdr_limits::open_on_dm_limit, "float")},
//     {"limits.LO_offload_limit",      make_nested_scalar_getter(&bdr_rtc_config::limits, &bdr_limits::LO_offload_limit, "float")},

//     // ----- camera (strings) -----
//     {"cam.fps",               make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::fps, "string")},
//     {"cam.gain",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::gain, "string")},
//     {"cam.testpattern",       make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::testpattern, "string")},
//     {"cam.bias",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::bias, "string")},
//     {"cam.flat",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::flat, "string")},
//     {"cam.imagetags",         make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::imagetags, "string")},
//     {"cam.led",               make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::led, "string")},
//     {"cam.events",            make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::events, "string")},
//     {"cam.extsynchro",        make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::extsynchro, "string")},
//     {"cam.rawimages",         make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::rawimages, "string")},
//     {"cam.cooling",           make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cooling, "string")},
//     {"cam.mode",              make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::mode, "string")},
//     {"cam.resetwidth",        make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::resetwidth, "string")},
//     {"cam.nbreadworeset",     make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::nbreadworeset, "string")},
//     {"cam.cropping",          make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cropping, "string")},
//     {"cam.cropping_columns",  make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cropping_columns, "string")},
//     {"cam.cropping_rows",     make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::cropping_rows, "string")},
//     {"cam.aduoffset",         make_nested_scalar_getter(&bdr_rtc_config::cam, &bdr_cam::aduoffset, "string")},

//     // ----- filters (uint8 masks + VectorXf DM projections) -----
//     {"filters.bad_pixel_mask",   make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::bad_pixel_mask, "vector<uint8>")},
//     {"filters.bad_pixel_mask_dm",make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::bad_pixel_mask_dm, "vector<float>")},
//     {"filters.pupil",            make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::pupil, "vector<uint8>")},
//     {"filters.pupil_dm",         make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::pupil_dm, "vector<float>")},
//     {"filters.secondary",        make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::secondary, "vector<uint8>")},
//     {"filters.secondary_dm",     make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::secondary_dm, "vector<float>")},
//     {"filters.exterior",         make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::exterior, "vector<uint8>")},
//     {"filters.exterior_dm",      make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::exterior_dm, "vector<float>")},
//     {"filters.inner_pupil_filt", make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::inner_pupil_filt, "vector<uint8>")},
//     {"filters.inner_pupil_filt_dm", make_nested_eigen_vector_getter(&bdr_rtc_config::filters, &bdr_filters::inner_pupil_filt_dm, "vector<float>")},

//     // ================= Derived/runtime (top-level) =================
//     {"zeroCmd",              make_eigen_vector_field_getter(&bdr_rtc_config::zeroCmd, "vector<double>")},
//     {"fps",                  make_scalar_field_getter(&bdr_rtc_config::fps, "double")},
//     {"gain",                 make_scalar_field_getter(&bdr_rtc_config::gain, "double")},
//     {"scale",                make_scalar_field_getter(&bdr_rtc_config::scale, "double")},
//     {"I2M_LO_runtime",       make_eigen_matrix_field_getter(&bdr_rtc_config::I2M_LO_runtime, "matrix<double>")},
//     {"I2M_HO_runtime",       make_eigen_matrix_field_getter(&bdr_rtc_config::I2M_HO_runtime, "matrix<double>")},
//     {"N0_dm_runtime",        make_eigen_vector_field_getter(&bdr_rtc_config::N0_dm_runtime, "vector<double>")},
//     {"I0_dm_runtime",        make_eigen_vector_field_getter(&bdr_rtc_config::I0_dm_runtime, "vector<double>")},
//     {"dark_dm_runtime",      make_eigen_vector_field_getter(&bdr_rtc_config::dark_dm_runtime, "vector<double>")},
//     {"sec_idx",              make_scalar_field_getter(&bdr_rtc_config::sec_idx, "int")},
//     {"m_s_runtime",          make_scalar_field_getter(&bdr_rtc_config::m_s_runtime, "double")},
//     {"b_s_runtime",          make_scalar_field_getter(&bdr_rtc_config::b_s_runtime, "double")},

//     // ----- runtime controllers (PIDController) -----
//     {"ctrl_LO.ctrl_type",    make_nested_scalar_getter(&bdr_rtc_config::ctrl_LO, &PIDController::ctrl_type, "string")},
//     {"ctrl_LO.kp",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::kp, "vector<double>")},
//     {"ctrl_LO.ki",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::ki, "vector<double>")},
//     {"ctrl_LO.kd",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::kd, "vector<double>")},
//     {"ctrl_LO.lower_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::lower_limits, "vector<double>")},
//     {"ctrl_LO.upper_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::upper_limits, "vector<double>")},
//     {"ctrl_LO.set_point",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::set_point, "vector<double>")},
//     {"ctrl_LO.output",       make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::output, "vector<double>")},
//     {"ctrl_LO.integrals",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::integrals, "vector<double>")},
//     {"ctrl_LO.prev_errors",  make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_LO, &PIDController::prev_errors, "vector<double>")},

//     {"ctrl_HO.ctrl_type",    make_nested_scalar_getter(&bdr_rtc_config::ctrl_HO, &PIDController::ctrl_type, "string")},
//     {"ctrl_HO.kp",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::kp, "vector<double>")},
//     {"ctrl_HO.ki",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::ki, "vector<double>")},
//     {"ctrl_HO.kd",           make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::kd, "vector<double>")},
//     {"ctrl_HO.lower_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::lower_limits, "vector<double>")},
//     {"ctrl_HO.upper_limits", make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::upper_limits, "vector<double>")},
//     {"ctrl_HO.set_point",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::set_point, "vector<double>")},
//     {"ctrl_HO.output",       make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::output, "vector<double>")},
//     {"ctrl_HO.integrals",    make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::integrals, "vector<double>")},
//     {"ctrl_HO.prev_errors",  make_nested_eigen_vector_getter(&bdr_rtc_config::ctrl_HO, &PIDController::prev_errors, "vector<double>")}
//     // --- NOTE: telemetry ring-buffers (telem.*) deliberately not exposed here ---
// };
// // --------------- Commander handlers: list + get (getter-only) ---------------

// nlohmann::json list_rtc_fields() {
//     nlohmann::json arr = nlohmann::json::array();
//     for (const auto& kv : RTC_FIELDS) {
//         arr.push_back({{"field", kv.first}, {"type", kv.second.type}});
//     }
//     return {{"ok", true}, {"fields", arr}};
// }

// // GET one field by "path" (e.g. "fps_override" or "ctrl_LO.kp")
// nlohmann::json get_rtc_field(std::string path) {
//     auto it = RTC_FIELDS.find(path);
//     if (it == RTC_FIELDS.end()) {
//         nlohmann::json keys = nlohmann::json::array();
//         for (const auto& kv : RTC_FIELDS) keys.push_back(kv.first);
//         return {{"ok", false}, {"error", "unknown field"}, {"field", path}, {"available", keys}};
//     }
//     const auto& fh = it->second;
//     return {{"ok", true}, {"field", path}, {"type", fh.type}, {"value", fh.get()}};
// }

// //// END rtc config file getter and setters ! 








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
    // assumes we are in a correct state to obtain reference ZWFS pupil - we do not checks 
    if (rtc_config.telem.img_dm.empty()) {
        std::cerr << "[I0_update] Warning: telemetry buffer is empty, cannot update I0." << std::endl;
        return;
    }

    // Initialize a sum vector with zeros, same size as first vector in img_dm
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(rtc_config.telem.img_dm.front().size());

    size_t count = 0;
    // read from the telem ring buffer
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

    // Subtraction of dark and bias was previously done in img_dm calc held in rtc telem ring buffer !!! 
    // but since Paranal AIV darks are subtracted in camera server so this internal subtraction is no longer used 

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

    // Compute the mean inside the good pupil
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

    // Replace exterior pixels to the mean of the interior - this is to avoid bad edge and division by zero issues
    for (Eigen::Index i = 0; i < P; ++i) {
        if (rtc_config.filters.inner_pupil_filt_dm(i) <= 0.7) {
            avg_dm(i) = inner_mean;
        }
    }

    // Save into runtime only
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

        // create the directory now if it doesn't exist
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


    m.def("configure_burst", cmd_configure_burst,  "Update the rolling burst window (size and dt) for non-destructive read mode slope estimates", "args"_arg);


    // uncomment and build July 2025 AIV
    m.def("change_boxcar", change_boxcar,  "Update the number of signal samples from telem ring buffer to weight an average. Only applied for values > 1", "args"_arg);


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

    //test in TTR115.0035 (not tested yet)
    m.def("poll_telem_vector", poll_telem_vector,
        "poll the second last entry to baldr's rolling telemetry buffers for the given field (must be a vector).\n"
        "Usage: poll_telem_vector u_HO\n",
        "telem fields : img, img_dm, signal, e_LO, u_LO, e_HO, u_HO, c_LO, c_HO"
        "args"_arg);

    // test in TTR115.0035 (not tested yet)
    m.def("poll_telem_scalar", poll_telem_scalar,
        "poll the second last entry to baldr's rolling telemetry buffers for the given field \n"
        "Usage: poll_telem_vector snr",
        "args"_arg);

    // m.def("list_rtc_fields", list_rtc_fields,
    //   "List exposed runtime config fields and types.");

    // m.def("get_rtc_field", get_rtc_field,
    //     "Get one runtime config field by path (string).",
    //     "args"_arg);
    m.def("list_rtc_fields", list_rtc_fields,
        "List exposed runtime config fields and types.",
        "args"_arg);

    m.def("get_rtc_field", get_rtc_field,
        "Get one runtime config field by path (string). Usage: get_rtc_field \"gain\"",
        "args"_arg);

    m.def("set_rtc_field", set_rtc_field,
        "Set one runtime config field by path and JSON value. Usage: set_rtc_field [\"ctrl_LO_config.kp\", [0.1,0.1,0.1]]",
        "args"_arg);

 }




std::map<std::string, std::string> parse_named_args(int argc, char* argv[]) {
    std::map<std::string, std::string> opts;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) != 0) {
            std::cerr << "[ERROR] Unexpected argument format: " << key << std::endl;
            continue;
        }

        key = key.substr(2);  // remove '--'
        if ((i + 1 < argc) && std::string(argv[i+1]).rfind("--", 0) != 0) {
            opts[key] = argv[++i];
        } else {
            opts[key] = "1";  // e.g., for --verbose
        }
    }
    return opts;
}


int main(int argc, char* argv[]) {
    auto args = parse_named_args(argc, argv);

    // Required
    if (!args.count("beam") || !args.count("mask")) {
        std::cerr << "Usage: " << argv[0]
                  << " --beam <id> --mask <name> [--mode bright|faint] [--config file.toml] [--socket tcp://*:6662]"
                  << std::endl;
        return 1;
    }

    int beam_id = std::stoi(args["beam"]);
    std::string phasemask = args["mask"];
    std::string observing_mode = args.count("mode") ? args["mode"] : "bright";

    if (observing_mode != "bright" && observing_mode != "faint") {
        std::cerr << "[ERROR] Invalid mode. Use 'bright' or 'faint'." << std::endl;
        return 1;
    }

    std::string filename;
    if (args.count("config")) {
        filename = args["config"];
        std::cout << "[INFO] Using user config file: " << filename << std::endl;
    } else {
        filename = find_latest_config_file(beam_id, observing_mode);
        std::cout << "[INFO] Using latest config: " << filename << std::endl;
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

        std::cout << "here " << std::endl;

        std::cout << "[INFO] RTC configuration initialized for beam " << beam_id << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing RTC config: " << e.what() << std::endl;
        return 1;
    }

    // Open shared memory
    if (!rtc_config.state.simulation_mode) {
        std::cout << "[INFO] Opening real SHM (not simulation)..." << std::endl;

        std::cout << "here in NOT simulation mode" << std::endl;

        // real hardware servers
        cam_host_str = "tcp://192.168.100.2:6667";
        mds_host_str = "tcp://192.168.100.2:5555";


    } else {
        //std::cerr << "[ERROR] Simulation mode not implemented yet." << std::endl;
        //throw std::runtime_error("Simulation mode not implemented");

        std::cout << "here in simulation mode" << std::endl;

        // Use the Fake servers!
        cam_host_str = "tcp://127.0.0.1:6667";
        mds_host_str = "tcp://127.0.0.1:5555";

    }

    // after we set the corrrect host addresses based on simulation mode (need to read in toml first)
    // then we init derived parameters (this requires zmq communication to camera server  - hence why we do it last)
    try {
        rtc_config.initDerivedParameters();
        std::cout << "[INFO] RTC configuration initialized for beam " << beam_id << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing RTC config: " << e.what() << std::endl;
        return 1;
    }
    //rtc_config.initDerivedParameters();


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


    // Start RTC threads
    std::thread rtc_thread(rtc);
    std::thread telemetry_thread(telemetry);

    std::cout << "[INFO] RTC and telemetry threads started." << std::endl;

    
    // prepare commander arguments
    std::vector<std::string> commander_args = {argv[0]};

    if (args.count("socket")){
        commander_args.push_back("--socket");
        commander_args.push_back(args["socket"]);
    }

    // this is a safe C style allocation of strings so if commander_args arn't allive we dont get invalid pointer
    std::vector<std::unique_ptr<char[]>> argv_storage;
    std::vector<char*> commander_argv;
    for (const auto& s : commander_args){
        argv_storage.emplace_back(std::make_unique<char[]>(s.size()+1));
        std::strcpy(argv_storage.back().get(), s.c_str());
        commander_argv.push_back(argv_storage.back().get());
    }
    
    // this was the old functional but maybe less robust way fo commander_argv
    //std::vector<char*> commander_argv;
    //for (auto& s : commander_args )
    //    commander_argv.push_back(&s[0]);
    
    int commander_argc = commander_argv.size();
    // Start commander server
    //commander::Server s(argc, argv);
    commander::Server s(commander_argc, commander_argv.data());

    // test we past commander input 
    std::cout << "commander server started" << std::endl;

    s.run();

    // Clean shutdown
    servo_mode = SERVO_STOP;
    rtc_thread.join();
    telemetry_thread.join();

    std::cout << "DONE" << std::endl;
    return 0;
}


// // previous working veruis 
// int main(int argc, char* argv[]) {
//     if (argc < 3 || argc > 5) {
//         std::cerr << "Usage: " << argv[0] << " <beam_id> <phaseMask> [mode (bright|faint)] [optional config_filename.toml]" << std::endl;
//         return 1;
//     }

//     // Mandatory inputs
//     beam_id = std::stoi(argv[1]);
//     phasemask = argv[2];

//     // Optional mode
//     if (argc >= 4) {
//         observing_mode = argv[3];
//         if (observing_mode != "bright" && observing_mode != "faint") {
//             std::cerr << "[ERROR] Invalid observing mode. Must be 'bright' or 'faint'." << std::endl;
//             return 1;
//         }
//     }

//     // Optional user configuration file
//     if (argc == 5) {
//         user_config_filename = argv[4];
//         user_provided_config = true;
//     }

//     std::string filename;
//     if (user_provided_config) {
//         filename = user_config_filename;
//         std::cout << "[INFO] Using user-provided config file: " << filename << std::endl;
//     } else {
//         try {
//             filename = find_latest_config_file(beam_id, observing_mode); // <--- updated find_latest_config_file
//             std::cout << "[INFO] Using latest config for mode '" << observing_mode << "': " << filename << std::endl;
//         } catch (const std::exception& e) {
//             std::cerr << "Error finding latest config file: " << e.what() << std::endl;
//             return 1;
//         }
//     }

//     // Load and parse TOML config
//     try {
//         config = toml::parse_file(filename);
//         std::cout << "Loaded configuration for beam " << beam_id << " from " << filename << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error parsing file " << filename << ": " << e.what() << std::endl;
//         return 1;
//     }

//     // Now set up rtc_config
//     std::string beamKey = "beam" + std::to_string(beam_id);
//     try {
//         rtc_config = readBDRConfig(config, beamKey, phasemask);
//         rtc_config.initDerivedParameters();
//         std::cout << "[INFO] RTC configuration initialized for beam " << beam_id << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error initializing RTC config: " << e.what() << std::endl;
//         return 1;
//     }

//     // Open shared memory
//     if (!rtc_config.state.simulation_mode) {
//         std::cout << "[INFO] Opening real SHM (not simulation)..." << std::endl;

//         ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id)).c_str());

//         std::string name = "dm" + std::to_string(beam_id) + "disp02";
//         std::string name0 = "dm" + std::to_string(beam_id);

//         if (ImageStreamIO_openIm(&dm_rtc, name.c_str()) != IMAGESTREAMIO_SUCCESS) {
//             std::cerr << "[ERROR] Failed to open DM SHM: " << name << std::endl;
//             return 1;
//         }
//         if (ImageStreamIO_openIm(&dm_rtc0, name0.c_str()) != IMAGESTREAMIO_SUCCESS) {
//             std::cerr << "[ERROR] Failed to open DM master SHM: " << name0 << std::endl;
//             return 1;
//         }

//     } else {
//         std::cerr << "[ERROR] Simulation mode not implemented yet." << std::endl;
//         throw std::runtime_error("Simulation mode not implemented");
//     }

//     // Start RTC threads
//     std::thread rtc_thread(rtc);
//     std::thread telemetry_thread(telemetry);

//     std::cout << "[INFO] RTC and telemetry threads started." << std::endl;

//     // Start commander server
//     commander::Server s(argc, argv);
//     s.run();

//     // Clean shutdown
//     servo_mode = SERVO_STOP;
//     rtc_thread.join();
//     telemetry_thread.join();

//     std::cout << "DONE" << std::endl;
//     return 0;
// }





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

