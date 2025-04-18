#define TOML_IMPLEMENTATION
#include "baldr.h"
#include <commander/commander.h>
#include <math.h> // For old-style C mathematics if needed.
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

// Forward declaration of global PID controllers 
//extern PIDController ctrl_LO;
//extern PIDController ctrl_HO; // this is now derived in rtc_config


// int servo_mode;
std::atomic<int> servo_mode; 
std::atomic<int> servo_mode_LO;
std::atomic<int> servo_mode_HO;
// int servo_mode_LO;
// int servo_mode_HO;
std::string telemFormat = "json";
//vector::<int> telescopes;

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


// int load_configuration(std::string pth){
//     std::cout << pth << std::endl;
//     return 1;
// }


//----------commander functions from here---------------


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
// json update_pid_param(json args) {
//     // Expect an array with exactly 4 elements:
//     // [controller, parameter_type, indices, value]
//     if (!args.is_array() || args.size() != 4) {
//         return json{{"error", "Expected an array with four elements: [controller, parameter_type, indices, value]"}};
//     }
//     try {
//         std::string controller_str = args.at(0).get<std::string>();
//         std::string parameter_type = args.at(1).get<std::string>();
//         std::string indices_str = args.at(2).get<std::string>();
//         double value = args.at(3).get<double>();

//         // Lock the PID mutex to protect concurrent access.
//         std::lock_guard<std::mutex> lock(ctrl_mutex);

//         // Select the appropriate PID controller.
//         PIDController* pid_ctrl = nullptr;
//         if (controller_str == "LO") {
//             pid_ctrl = &ctrl_LO;
//         } else if (controller_str == "HO") {
//             pid_ctrl = &ctrl_HO;
//         } else {
//             return json{{"error", "Invalid controller type: " + controller_str + ". Must be \"LO\" or \"HO\"."}};
//         }
        
//         // Choose the target parameter vector. For parameters that are represented as Eigen::VectorXd, 
//         // we use a pointer to that vector.
//         Eigen::VectorXd* target_vector = nullptr;
//         if (parameter_type == "kp") {
//             target_vector = &pid_ctrl->kp;
//         } else if (parameter_type == "ki") {
//             target_vector = &pid_ctrl->ki;
//         } else if (parameter_type == "kd") {
//             target_vector = &pid_ctrl->kd;
//         } else if (parameter_type == "set_point") {
//             target_vector = &pid_ctrl->set_point;
//         } else if (parameter_type == "lower_limits") {
//             target_vector = &pid_ctrl->lower_limits;
//         } else if (parameter_type == "upper_limits") {
//             target_vector = &pid_ctrl->upper_limits;
//         } else {
//             return json{{"error", "Invalid parameter type: " + parameter_type + ". Use one of \"kp\", \"ki\", \"kd\", \"set_point\", \"lower_limits\", or \"upper_limits\"."}};
//         }
        
//         int n = target_vector->size();
//         if (indices_str == "all") {
//             target_vector->setConstant(value);
//             std::cout << "Updated all elements of " << parameter_type << " in controller " << controller_str 
//                       << " to " << value << std::endl;
//         } else {
//             // Use your split_indices helper function to split indices_str by commas.
//             std::vector<int> idxs = split_indices(indices_str);
//             if (idxs.empty()) {
//                 return json{{"error", "No valid indices provided"}};
//             }
//             for (int idx : idxs) {
//                 if (idx < 0 || idx >= n) {
//                     return json{{"error", "Index " + std::to_string(idx) + " out of range for " + parameter_type 
//                                         + " vector of size " + std::to_string(n)}};
//                 }
//                 (*target_vector)(idx) = value;
//             }
//             std::cout << "Updated indices (" << indices_str << ") of " << parameter_type << " in controller " 
//                       << controller_str << " with value " << value << std::endl;
//         }
//         return json{{"status", "success"}};
//     }
//     catch (const std::exception &ex) {
//         return json{{"error", ex.what()}};
//     }
// }

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
// json print_pid_attribute(json args) {
//     // Expect exactly two elements: [controller, attribute]
//     if (!args.is_array() || args.size() != 2) {
//         return json{{"error", "Expected an array with two elements: [controller, attribute]"}};
//     }
    
//     try {
//         std::string controller_str = args.at(0).get<std::string>();
//         std::string attribute = args.at(1).get<std::string>();
        
//         PIDController* ctrl = nullptr;
//         if (controller_str == "LO") {
//             ctrl = &ctrl_LO;
//         } else if (controller_str == "HO") {
//             ctrl = &ctrl_HO;
//         } else {
//             return json{{"error", "Invalid controller specified. Must be \"LO\" or \"HO\""}};
//         }
        
//         // Helper lambda to convert an Eigen::VectorXd to a std::vector<double>
//         auto eigen_to_vector = [](const Eigen::VectorXd &v) -> std::vector<double> {
//             return std::vector<double>(v.data(), v.data() + v.size());
//         };
        
//         // Compare the attribute string and return the corresponding value.
//         if (attribute == "kp") {
//             return json{{"kp", eigen_to_vector(ctrl->kp)}};
//         } else if (attribute == "ki") {
//             return json{{"ki", eigen_to_vector(ctrl->ki)}};
//         } else if (attribute == "kd") {
//             return json{{"kd", eigen_to_vector(ctrl->kd)}};
//         } else if (attribute == "lower_limits") {
//             return json{{"lower_limits", eigen_to_vector(ctrl->lower_limits)}};
//         } else if (attribute == "upper_limits") {
//             return json{{"upper_limits", eigen_to_vector(ctrl->upper_limits)}};
//         } else if (attribute == "set_point") {
//             return json{{"set_point", eigen_to_vector(ctrl->set_point)}};
//         } else if (attribute == "output") {
//             return json{{"output", eigen_to_vector(ctrl->output)}};
//         } else if (attribute == "integrals") {
//             return json{{"integrals", eigen_to_vector(ctrl->integrals)}};
//         } else if (attribute == "prev_errors") {
//             return json{{"prev_errors", eigen_to_vector(ctrl->prev_errors)}};
//         } else {
//             return json{{"error", "Unknown attribute: " + attribute}};
//         }
//     } catch (const std::exception &ex) {
//         return json{{"error", ex.what()}};
//     }
// }




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


COMMANDER_REGISTER(m)
{
    using namespace commander::literals;

    // You can register a function or any other callable object as
    // long as the signature is deductible from the type.
    // m.def("load_configuration", load_configuration, "Load a configuration file. Return true if successful.", 
    //     "filename"_arg="def.toml");

    

    m.def("stop_baldr", stop_baldr,
          "stop the baldr rtc loop","mode"_arg);

    m.def("close_baldr_LO", close_baldr_LO,
          "close the LO servo loop for baldr","mode"_arg);

    m.def("open_baldr_LO", open_baldr_LO,
          "open the LO servo loop for baldr - resetting gains and flatten DM","mode"_arg);

    m.def("close_baldr_HO", close_baldr_HO,
          "close the HO servo loop for baldr","mode"_arg);

    m.def("open_baldr_HO", open_baldr_HO,
          "open the HO servo loop for baldr - resetting gains and flatten DM","mode"_arg);

    m.def("save_telemetry", save_telemetry,
          "dump telemetry in the circular buffer to file","mode"_arg);

    m.def("set_telem_capacity", set_telem_capacity,
          "Set the capacity for all telemetry ring buffers (how many samples we hold). e.g: set_telem_capacity [200]",
          "args"_arg);

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

    //print_pid_attribute ["HO","ki"], print_pid_attribute ["LO","kp"]
    m.def("print_pid_attribute", print_pid_attribute,
          "Print the specified attribute of a PID controller.\n"
          "Usage: print_pid_attribute [controller, attribute]\n"
          "  - controller: \"LO\" or \"HO\"\n"
          "  - attribute: one of \"kp\", \"ki\", \"kd\", \"lower_limits\", \"upper_limits\", \"set_point\", "
          "\"output\", \"integrals\", or \"prev_errors\"",
          "args"_arg);

    //e.g. reload_config ["new_config.toml", "H5"]
    m.def("reload_config", reload_config,
          "Reload a new configuration file with a specified phasemask. It must be .toml \n"
          "from the baldr calibration pipeline. Also assumes the same beam_id as is currently configured.\n"
          "Usage: reload_config [\"new_filename.toml\", \"new_mask\"]",
          "args"_arg);

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
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <beam_id1> <phaseMask1>" << std::endl;
        return 1;
    }
    
    beam_id = std::stoi(argv[1]);
    phasemask = argv[2];
    // Compute the configuration filename and parse the TOML file.
    std::string filename = "baldr_config_" + std::to_string(beam_id) + ".toml";
    try {
        config = toml::parse_file(filename);
        std::cout << "Loaded configuration for beam " << beam_id << " from " << filename << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing file " << filename << ": " << e.what() << std::endl;
        return 1;
    }

    //to create a global list of bdr_rtc_config structs:
    std::string beamKey = "beam" + std::to_string(beam_id);
    try {
        rtc_config = readBDRConfig(config, beamKey, phasemask);
        std::cout << "after read in baldr main rtc.ctrl_LO_config.kp.size() = " << rtc_config.ctrl_LO_config.kp.size() << std::endl;
        rtc_config.initDerivedParameters();
        std::cout << "after initDerived parameters in baldr main rtc.ctrl_LO_config.kp.size() = " << rtc_config.ctrl_LO_config.kp.size() << std::endl;

        std::cout << "Initialized configuration for beam " << beam_id << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing configuration for beam " << beam_id << ": " << e.what() << std::endl;
        return 1;
    }

    // The C-red image for the baldr subarray is /dev/shm/baldrN.im.shm, 
    // where N is the beam number.
    // 
    //     ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id) ).c_str());
        
    //     // Open the DM image. It is e.g. /dev/shm/dm2disp02.im.shm for beam 2.
    //     std::string dm_filename = "dm" + std::to_string(beam_id) + "disp02" ;
    //     ImageStreamIO_openIm(&dm_rtc, dm_filename.c_str());
    // 
    if (!rtc_config.state.simulation_mode) {
        std::cout << "SHM configuration not in simulation mode" << std::endl;
        // Real mode: open real images from the shared memory.
        ImageStreamIO_openIm(&subarray, ("baldr" + std::to_string(beam_id)).c_str());
        
        // Open the DM image. (For beam 2, for example, this may be: "/dev/shm/dm2disp02.im.shm")
        std::string dm_filename = "dm" + std::to_string(beam_id) + "disp02";
        ImageStreamIO_openIm(&dm_rtc, dm_filename.c_str());

        //add.
        ImageStreamIO_semflush(&dm_rtc, /*index=*/-1);

    } else {
        // Simulation mode: if not yet implemented, raise an error.
        std::cerr << "Simulation mode not implemented. Aborting." << std::endl;
        throw std::runtime_error("Simulation mode not implemented");
        
        // Alternatively, if you prefer to use dummy simulation images,
        // you might do something like:
        // ImageStreamIO_openIm(&subarray, "simulated_baldr.im.shm");
        // ImageStreamIO_openIm(&dm_rtc, "simulated_dm.im.shm");
    }
    // Start the main RTC and telemetry threads. 


    std::thread rtc_thread(rtc);
    std::thread telemetry_thread(telemetry);

    std::cout << "after rtc thread started in  main rtc.ctrl_LO_config.kp.size() = " << rtc_config.ctrl_LO_config.kp.size() << std::endl;
    // Initialize the commander server and run it
    commander::Server s(argc, argv);
    s.run();

    // Join the  thread
    servo_mode = SERVO_STOP;
    rtc_thread.join();
    telemetry_thread.join();

    // Continue with your RTC main loop using rtc_config_list...
    std::cout << "DONE" << std::endl;
    
    return 0;
}





/// TO TEST ZMQ COMMUNICATION TO CAMERA WITHIN RTC (FOR CHECKING STATE - PARAMETERS ARE NORMALIZED ADU/s SO NEED TO CONVERT TO ADU)
// req_client.cpp
// #include <zmq.hpp>
// #include <string>
// #include <iostream>

// int main() {
//     // Create a ZeroMQ context with one I/O thread.
//     zmq::context_t context(1);
    
//     // Create a REQ (request) socket.
//     zmq::socket_t socket(context, zmq::socket_type::req);
    
//     // Connect to the server (adjust hostname and port as needed).
//     std::string endpoint = "tcp://localhost:5555";
//     socket.connect(endpoint);
//     std::cout << "Client: Connected to " << endpoint << std::endl;
    
//     // Create a request message.
//     std::string request_str = "Hello from C++ REQ";
//     zmq::message_t request(request_str.data(), request_str.size());
    
//     // Send the request (this puts the socket in a send-wait state).
//     std::cout << "Client: Sending request..." << std::endl;
//     socket.send(request, zmq::send_flags::none);
    
//     // Now wait for and receive the reply.
//     zmq::message_t reply;
//     // Note: The socket must be in the receive state now.
//     auto res = socket.recv(reply, zmq::recv_flags::none);
//     if (res) {
//         std::string reply_str(static_cast<char*>(reply.data()), reply.size());
//         std::cout << "Client: Received reply: " << reply_str << std::endl;
//     } else {
//         std::cerr << "Client: No reply received!" << std::endl;
//     }
    
//     return 0;
// }