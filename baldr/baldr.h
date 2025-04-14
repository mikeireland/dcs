#include <ImageStreamIO.h>
#include <stdlib.h>
#include <iostream>
#define TOML_HEADER_ONLY 0
#include <toml.hpp>
#include <mutex>
#include <thread>
#include <boost/circular_buffer.hpp>

//#include <string>
//#include <toml++/toml.h>
#include <Eigen/Dense>
#include <vector>
#include <cstdint>
#include <stdexcept>


//----------Defines-----------

#define N_TEL 4 // Number of telescopes
#define N_BL 6  // Number of baselines
#define N_CP 4  // Number of closure phases

// Different types of servo modes.
#define SERVO_PID 0 // PID servo mode
#define SERVO_STOP 2

//----------Constant Arrays-----------

//----- Structures and typedefs------

// just putting everything here for now - will move to commander when blantely obvious that I need to


//-------Commander structs-------------



// Converts a TOML array (2D array of numbers) to an Eigen matrix.
//Eigen::MatrixXd convertTomlArrayToEigenMatrix(const toml::array& arr);
//convertTomlArrayToEigenMatrix(const toml::array& arr, const Derived& /*dummy*/)
// Converts a TOML array (2D array of booleans) to an Eigen boolean matrix.
//Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> convertTomlArrayToBoolMatrix(const toml::array& arr);


// Template conversion function that takes a TOML array and a dummy
// variable of the desired Eigen type. The dummy parameter is only used
// to deduce the return type. This is useful because we pre-define our struct types and just need to 
// populate them correctly. 
// Also added extra field name at the end which checks size of input (toml) vector/matrix 
// against what is expected according to the struct. If inconsistent it fails and tells you
// precisely where and why (what the size mismatch is)
template<typename Derived>
Derived convertTomlArrayToEigenMatrix(const toml::array& arr, const Derived& /*dummy*/, const std::string& fieldName = "") {
    using Scalar = typename Derived::Scalar;
    size_t rows = arr.size();
    size_t cols = 1;
    if (rows > 0 && arr[0].as_array()) {
        cols = arr[0].as_array()->size();
    }
    
    if(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::RowsAtCompileTime != static_cast<int>(rows)) {
        std::ostringstream oss;
        oss << "Field \"" << fieldName << "\": Expected " << Derived::RowsAtCompileTime 
            << " rows but got " << rows;
        throw std::runtime_error(oss.str());
    }
    if(Derived::ColsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != static_cast<int>(cols)) {
        std::ostringstream oss;
        oss << "Field \"" << fieldName << "\": Expected " << Derived::ColsAtCompileTime 
            << " cols but got " << cols;
        throw std::runtime_error(oss.str());
    }
    
    Derived result;
    result.resize(rows, cols);
    if (cols == 1) {
        for (size_t i = 0; i < rows; ++i) {
            auto val_opt = arr[i].value<double>();
            result(i, 0) = val_opt ? static_cast<Scalar>(*val_opt) : Scalar(0);
        }
    } else {
        for (size_t i = 0; i < rows; ++i) {
            const auto* rowArr = arr[i].as_array();
            if (rowArr) {
                for (size_t j = 0; j < cols; ++j) {
                    auto val_opt = rowArr->at(j).value<double>();
                    result(i, j) = val_opt ? static_cast<Scalar>(*val_opt) : Scalar(0);
                }
            }
        }
    }
    return result;
}


// Declaration of readConfig function
toml::table readConfig(const std::string &filename);

//bdr_rtc_config readBDRConfig(const toml::table& config, const std::string& beamKey, const std::string& phaseKey);

//-----------------------------------------------------
// bdr_method: top-level method definitions.
struct bdr_state {
    std::string DM_flat;          // e.g., "baldr"
    std::string signal_space;   // where we consider our signals (pixel or dm)
    int LO;                       // low-order modes (e.g., 2)
    std::string controller_type;  // e.g., "PID"
    std::string inverse_method;   // e.g., "map"
    std::string phasemask; // what phasemask are we using 
    int auto_close; // do we close loops automatically? 
    int auto_open;// do we open loops automatically?  
    int auto_tune; // do we automatically tune gains?
    int take_telemetry; // do we automatically tune gains?
    int simulation_mode ;// simulatie camera and DM 
    // Validate method.
    void validate() const {
        if (DM_flat.empty())
            throw std::runtime_error("bdr_state: DM_flat is empty.");
        if (LO <= 0)
            throw std::runtime_error("bdr_state: LO must be positive.");
        if (controller_type.empty())
            throw std::runtime_error("bdr_state: controller_type is empty.");
        if (inverse_method.empty())
            throw std::runtime_error("bdr_state: inverse_method is empty.");
    }
};

//-----------------------------------------------------
// bdr_reduction: Beam-specific reduction products.
struct bdr_reduction {
    Eigen::VectorXd bias;      // Mean (flattened) bias frame (ADU)
    Eigen::VectorXd bias_dm;   // Bias intensity interpolated to DM pixels (ADU)
    Eigen::VectorXd dark;      // Mean (flattened) dark frame (ADU/s/gain)
    Eigen::VectorXd dark_dm;   // Dark intensity interpolated to DM pixels (ADU/s/gain)

    void project_to_dm( Eigen::MatrixXd I2A ){
        bias_dm = I2A * bias; 
        dark_dm = I2A * dark; 
    }

    void validate() const {
        if (bias.size() == 0 || bias_dm.size() == 0 || dark.size() == 0 || dark_dm.size() == 0)
            throw std::runtime_error("bdr_reduction: One or more vectors are empty.");
        // You might want to check that bias_dm has the same size as bias, etc.
    }
};

//-----------------------------------------------------
// bdr_pixels: Pixel indices.
struct bdr_pixels {
    Eigen::Matrix<int16_t, Eigen::Dynamic, 1> crop_pixels; // r1, r2, c1, c2 for cropping.
    Eigen::Matrix<int32_t, Eigen::Dynamic, 1> bad_pixels;      // Indices (flattened)
    Eigen::Matrix<int32_t, Eigen::Dynamic, 1> pupil_pixels;      // Pupil pixel indices.
    Eigen::Matrix<int32_t, Eigen::Dynamic, 1> interior_pixels;   // Interior pupil indices.
    Eigen::Matrix<int32_t, Eigen::Dynamic, 1> secondary_pixels;  // Secondary pixel indices.
    Eigen::Matrix<int32_t, Eigen::Dynamic, 1> exterior_pixels;   // Exterior pixel indices.

    void validate() const {
        // You can add size checks or consistency tests.
        if (crop_pixels.size() != 4)
            throw std::runtime_error("bdr_pixels: crop_pixels must have 4 elements.");
    }
};

//-----------------------------------------------------
// bdr_refence_pupils: Reference intensities.
struct bdr_refence_pupils {
    Eigen::VectorXd I0;         // Reduced reference intensity (ADU) [flattened]
    Eigen::VectorXd N0;         // Reduced reference intensity (ADU)
    Eigen::VectorXd norm_pupil; // Filtered reference intensity (ADU)
    Eigen::VectorXd norm_pupil_dm; // Interpolated to DM pixels (ADU)
    Eigen::VectorXd I0_dm;      // Interpolated to DM pixels (ADU)
    

    void project_to_dm( Eigen::MatrixXd I2A ){
        norm_pupil_dm = I2A * norm_pupil; 
        I0_dm = I2A * I0; 
    }


    void validate() const {
        if (I0.size() == 0 || N0.size() == 0 || norm_pupil.size() == 0)
            throw std::runtime_error("bdr_refence_pupils: one or more vectors are empty.");
    }
};

//-----------------------------------------------------
// bdr_matricies: Matrices for processing.
struct bdr_matricies {
    Eigen::MatrixXd I2A;      // Interpolation matrix (pixels to DM)
    Eigen::MatrixXd I2M;      // Intensity-to-mode projection matrix.
    Eigen::MatrixXd I2M_LO;   // For low-order modes.
    Eigen::MatrixXd I2M_HO;   // For high-order modes.
    Eigen::MatrixXd M2C;      // Mode-to-DM command projection matrix.
    Eigen::MatrixXd M2C_LO;   // LO mode to DM command.
    Eigen::MatrixXd M2C_HO;   // HO mode to DM command.
    Eigen::MatrixXd I2rms_sec; // For secondary obstruction.
    Eigen::MatrixXd I2rms_ext; // For exterior pupil.
    // Additional size variables:
    int32_t szm; // Number of modes.
    int32_t sza; // Number of actuators.
    int32_t szp; // Number of pixels.

    void validate() const {
        // Example: check that dimensions are consistent.
        if (I2A.rows() == 0 || I2A.cols() == 0)
            throw std::runtime_error("bdr_matricies: I2A matrix is empty.");
        // Further consistency checks can be added.
    }
};

//-----------------------------------------------------
// bdr_controller: AO controller settings.
struct bdr_controller {
    // For now, only gains. In a complete system you might add anti-windup limits, etc.
    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    Eigen::VectorXd lower_limits;
	Eigen::VectorXd upper_limits;
	Eigen::VectorXd set_point;
    void validate() const {
        if (kp.size() == 0 || ki.size() == 0 || kd.size() == 0)
            throw std::runtime_error("bdr_controller: one or more gain vectors are empty.");
        if ( (kp.size() != ki.size()) || (kp.size() != kd.size()))
            throw std::runtime_error("bdr_controller: gain vector sizes mismatch.");
    }
    // Constructor with default vector size of 5.
    bdr_controller(int vsize = 140)
        : kp(Eigen::VectorXd::Zero(vsize)),
          ki(Eigen::VectorXd::Zero(vsize)),
          kd(Eigen::VectorXd::Zero(vsize)),
          lower_limits(Eigen::VectorXd::Constant(vsize, -42)),
          upper_limits(Eigen::VectorXd::Constant(vsize, 42)),
          set_point(Eigen::VectorXd::Zero(vsize))
        {}
};

//-----------------------------------------------------
// bdr_limits: Limits and conditions for loop control.
struct bdr_limits {
    float close_on_strehl_limit; // when should we automatically try to close the loop?
    float open_on_strehl_limit; // at what level of Strehl (estimate) do we open the loop?
    float open_on_flux_limit; // at what level of flux do we open the loop?
    float open_on_dm_limit; // at what level of DM (rms? or peaktovalley) do we open the loop (or limit gains)?
    float LO_offload_limit; // at what level do we try offload the lower order modes ?

    void validate() const {

    }
};



//-----------------------------------------------------
// bdr_cam: Camera configuration.
struct bdr_cam {
    std::string fps;
    std::string gain;
    std::string testpattern;
    std::string bias;
    std::string flat;
    std::string imagetags;
    std::string led;
    std::string events;
    std::string extsynchro;
    std::string rawimages;
    std::string cooling;
    std::string mode;
    std::string resetwidth;
    std::string nbreadworeset;
    std::string cropping;
    std::string cropping_columns;
    std::string cropping_rows;
    std::string aduoffset;
    
    void validate() const {
        // Check required fields; for now, we assume they all must be non-empty.
        if (fps.empty() || gain.empty())
            throw std::runtime_error("bdr_cam: fps and gain must be provided.");
    }
};

//-----------------------------------------------------
// // bdr_telem: Telemetry configuration.
struct bdr_telem {
    boost::circular_buffer<Eigen::VectorXd> timestamp; // if you want to store each sample’s timestamp
    boost::circular_buffer<Eigen::VectorXd> img;       // each entry is a vector (e.g. flattened image)
    boost::circular_buffer<Eigen::VectorXd> signal;    
    boost::circular_buffer<Eigen::VectorXd> e_TT;
    boost::circular_buffer<Eigen::VectorXd> u_TT;
    boost::circular_buffer<Eigen::VectorXd> e_HO;
    boost::circular_buffer<Eigen::VectorXd> u_HO;
    boost::circular_buffer<Eigen::VectorXd> dm_ch0;
    boost::circular_buffer<Eigen::VectorXd> dm_ch1;
    boost::circular_buffer<Eigen::VectorXd> dm_ch2;
    boost::circular_buffer<Eigen::VectorXd> dm_ch3;
    boost::circular_buffer<Eigen::VectorXd> dm;
    float strehl_est;
    float dm_rms;

    // Constructor that sets a fixed capacity (for example, 100 samples)
    bdr_telem(size_t capacity = 100)
      : timestamp(capacity),
        img(capacity),
        signal(capacity),
        e_TT(capacity),
        u_TT(capacity),
        e_HO(capacity),
        u_HO(capacity),
        dm_ch0(capacity),
        dm_ch1(capacity),
        dm_ch2(capacity),
        dm_ch3(capacity),
        dm(capacity),
        strehl_est(0),
        dm_rms(0)
    {}
    
    void validate() const {
        // Add any validation if needed.
    }
};

// struct bdr_telem {
//     // Define telemetry fields as needed.
//     std::vector<std::chrono::steady_clock::time_point> timestamp;
//     std::vector<int16_t> img;
//     std::vector<double> signal; 
//     std::vector<double> e_TT;
//     std::vector<double> u_TT;
//     std::vector<double> e_HO;
//     std::vector<double> u_HO;
//     std::vector<double> dm_ch0;
//     std::vector<double> dm_ch1;
//     std::vector<double> dm_ch2;
//     std::vector<double> dm_ch3; 
//     std::vector<double> dm;
//     float strehl_est;
//     float dm_rms;


//     bdr_telem(){
//         timestamp =  {}; //std::chrono::steady_clock::now();
//         img  = {};
//         signal  = {};
//         e_TT  = {};
//         u_TT  = {};
//         e_HO  = {};
//         u_HO  = {};
//         dm_ch0  = {};
//         dm_ch1  = {};
//         dm_ch2  = {};
//         dm_ch3   = {};
//         dm = {};
//         strehl_est = 0 ;
//         dm_rms = 0;
//     }

    
//     void validate() const {
//         // Could check for non-empty parameters if required.
//     }
// };

//-----------------------------------------------------
// bdr_filters: Boolean masks.
struct bdr_filters {
    Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> bad_pixel_mask;
    Eigen::VectorXf bad_pixel_mask_dm;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> pupil;
    Eigen::VectorXf pupil_dm;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> secondary;
    Eigen::VectorXf secondary_dm;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> exterior;
    Eigen::VectorXf exterior_dm;
    Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> inner_pupil_filt;
    Eigen::VectorXf inner_pupil_filt_dm;

    void validate() const {
        }

    // Project all masks to DM space.
    void project_to_dm(const Eigen::MatrixXd& I2A) {
        std::cout << "I2A.cols() = " << I2A.cols() << std::endl;
        std::cout << "bad_pixel_mask.size() = " <<  bad_pixel_mask.size() << std::endl;
        // Do similar prints for pupil, secondary, etc.
        // Assume 'mask' is one of your Eigen vector masks, e.g., exterior.
        Eigen::VectorXd mask_d = exterior.cast<double>();

        // Debug prints: Print I2A number of columns and the mask size.
        std::cout << "I2A dimensions: " << I2A.rows() << " x " << I2A.cols() << std::endl;
        std::cout << "Mask (exterior) size: " << mask_d.size() << std::endl;

        // Convert each mask to a double vector, multiply by I2A, then cast to float.
        bad_pixel_mask_dm = (I2A * bad_pixel_mask.cast<double>()).cast<float>();
        pupil_dm = (I2A * pupil.cast<double>()).cast<float>();
        secondary_dm = (I2A * secondary.cast<double>()).cast<float>();
        exterior_dm = (I2A * exterior.cast<double>()).cast<float>();
        inner_pupil_filt_dm = (I2A * inner_pupil_filt.cast<double>()).cast<float>();
    }

};

//-----------------------------------------------------
// Master configuration struct for RTC.
struct bdr_rtc_config {
    bdr_state state;
    bdr_reduction reduction;
    bdr_pixels pixels;
    bdr_refence_pupils reference_pupils;
    bdr_matricies matrices;
    //bdr_controller controller;
    ////heree
    bdr_controller ctrl_LO_config;
    bdr_controller ctrl_HO_config;
    bdr_limits limits;
    bdr_cam cam;
    bdr_telem telem;
    bdr_filters filters;


    void validate() const {
        state.validate();
        reduction.validate();
        pixels.validate();
        reference_pupils.validate();
        matrices.validate();
        //heree
        //controller.validate();
        ctrl_LO_config.validate();
        ctrl_HO_config.validate();
        limits.validate();
        cam.validate();
        telem.validate();
        filters.validate();
        
    }
};


bdr_rtc_config readBDRConfig(const toml::table& config, const std::string& beamKey, const std::string& phaseKey);



// Declaration of the PIDController class.
class PIDController {
public:
    // Constructors.
    PIDController(const Eigen::VectorXd& kp_in,
                  const Eigen::VectorXd& ki_in,
                  const Eigen::VectorXd& kd_in,
                  const Eigen::VectorXd& lower_limit_in,
                  const Eigen::VectorXd& upper_limit_in,
                  const Eigen::VectorXd& setpoint_in);
    PIDController(const bdr_controller& config_in);
    PIDController();

    // Process the measured input to produce control output.
    Eigen::VectorXd process(const Eigen::VectorXd& measured);

    // Utility functions.
    void set_all_gains_to_zero();
    void reset();

    // Public members 
    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    Eigen::VectorXd lower_limits;
    Eigen::VectorXd upper_limits;
    Eigen::VectorXd set_point;
    std::string ctrl_type;
    Eigen::VectorXd output;
    Eigen::VectorXd integrals;
    Eigen::VectorXd prev_errors;
};

// An encoded 2D image in row-major form.
// If this is to be useful, then the heimdallr EncodedImage
// should become a library.
struct EncodedImage
{
    unsigned int szx, szy;
    std::string type;
    std::string message;
};




//-------End of Commander structs------



// -------- Extern global definitions ------------
// The static initial input parameters
//extern toml::table config;
//extern std::vector<toml::table> config; // initial configuration 
extern int beam_id; 
extern std::string phasemask;
extern toml::table config;
extern bdr_rtc_config rtc_config;


//extern std::vector<bdr_rtc_config> rtc_config_list; // what the rtc will use and edit
extern int servo_mode;
// extern vector::<int> telescopes;

// Servo parameters. These are the parameters that will be adjusted by the commander

// We at least need a mutex for RTC parameters.
extern std::mutex rtc_mutex;

// The C-Red Image subarray and DM
extern IMAGE subarray;
extern IMAGE dm_rtc;

// Main thread function for the RTC
void rtc();

// Main thread function for the telemetry
void telemetry();
