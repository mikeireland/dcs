#include "baldr.h"
#include <chrono>
#include <iostream>
#include <ImageStreamIO.h>
#include <cstdint>  // for uint16_t
#include <nlohmann/json.hpp>

#include <fstream>


using json = nlohmann::json;

// json telemetry_to_json(const bdr_telem &telemetry) {
//     json j;
//     // Convert timestamp to milliseconds since epoch.
//     // Convert the vector of steady_clock time_points to a JSON array in milliseconds.
//     json ts = json::array();
//     for (const auto &t : telemetry.timestamp) {
//         // Convert the steady_clock::time_point to milliseconds.
//         auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
//         ts.push_back(ms);
//     }
//     j["timestamps"] = ts;   // Use "timestamps" since it's an array.
    
//     j["img"] = telemetry.img;                // nlohmann::json converts std::vector easily.
//     j["signal"] = telemetry.signal;
//     j["e_TT"] = telemetry.e_TT;
//     j["u_TT"] = telemetry.u_TT;
//     j["e_HO"] = telemetry.e_HO;
//     j["u_HO"] = telemetry.u_HO;
//     j["dm_ch0"] = telemetry.dm_ch0;
//     j["dm_ch1"] = telemetry.dm_ch1;
//     j["dm_ch2"] = telemetry.dm_ch2;
//     j["dm_ch3"] = telemetry.dm_ch3;
//     j["dm"] = telemetry.dm;
//     j["strehl_est"] = telemetry.strehl_est;
//     j["dm_rms"] = telemetry.dm_rms;
//     return j;
// }

// void write_telemetry_to_json(const bdr_telem &telemetry, const std::string &filename) {
//     json j = telemetry_to_json(telemetry);
//     std::ofstream ofs(filename);
//     if (!ofs) {
//         std::cerr << "Could not open file " << filename << " for writing." << std::endl;
//         return;
//     }
//     ofs << j.dump(4); // pretty-print with an indent of 4 spaces.
//     ofs.close();
//     std::cout << "Telemetry written to " << filename << std::endl;
// }



long unsigned int rtc_cnt=0;
unsigned int nerrors=0;

// Add local (to the RTC) variables here

// controllers - these get initialized by the rtc.ctrl_LO_config and rtc.ctrl_HO_config struct
// PIDController ctrl_LO ; 
// PIDController ctrl_HO ; 

// intermediate products 
Eigen::VectorXd img;

Eigen::VectorXd img_dm;

Eigen::VectorXd sig;

Eigen::VectorXd e_HO;
Eigen::VectorXd u_HO;

Eigen::VectorXd e_LO;
Eigen::VectorXd u_LO;

Eigen::VectorXd c_LO;
Eigen::VectorXd c_HO;
Eigen::VectorXd dmCmd;

// dm rms model
double dm_rms_est_1 ; // from secondary obstruction
double dm_rms_est_2 ; // from exterior pixels 


//------------------------------------------------------------------------------
// Drain any outstanding semaphore “posts” so that
// the next semwait() really waits for a fresh frame.
//------------------------------------------------------------------------------
static inline void catch_up_with_sem(IMAGE* img, int semid) {
    // keep grabbing until there are no more pending posts
    while (ImageStreamIO_semtrywait(img, semid) == 0) { /* nothing just do it*/; }
}

/**
 Reads a frame from shared memory which should match the expected number of pixels expectedPixels.
**/
Eigen::VectorXd getFrameFromSharedMemory(int expectedPixels) {
    if (!subarray.md) {
        //ImageStreamIO_destroyIm(&subarray);
        throw std::runtime_error("No metadata found in shared memory image.");
    }
    
    int nx = subarray.md->size[0];
    int ny = (subarray.md->naxis > 1) ? subarray.md->size[1] : 1;
    int totalPixels = nx * ny;
    
    if (totalPixels != expectedPixels) {
        //ImageStreamIO_destroyIm(&subarray);
        throw std::runtime_error("Frame size mismatch: expected " +
                                 std::to_string(expectedPixels) +
                                 ", got " + std::to_string(totalPixels));
    }
    
    // Assume the image data is stored as 16-bit unsigned ints.
    ImageStreamIO_semwait( &subarray, 1);  // waiting for image update
    uint16_t* data = subarray.array.UI16;
    if (!data) {
        //ImageStreamIO_destroyIm(&subarray);
        throw std::runtime_error("Data pointer in shared memory is null.");
    }
    
    // Convert the pixel data to an Eigen vector.
    Eigen::VectorXd frame(totalPixels);
    for (int i = 0; i < totalPixels; ++i) {
        frame(i) = static_cast<double>(data[i]);
    }
    
    // Clean up the mapping.
    // ImageStreamIO_destroyIm(&subarray);
    
    return frame;
}


/**                     
writes to DM SHM.
**/
void updateDMSharedMemory(const Eigen::VectorXd &dmCmd) {

    // Get image parameters (assumed constant during runtime)
    //int dm_naxis = dm_rtc.md->naxis;
    //int dm_width  = dm_rtc.md->size[0];
    //int dm_height = (naxis > 1) ? dm_rtc.md->size[1] : 1;
    int dm_totalElements = dm_rtc.md->nelement; // e.g., 1600 for 40x40
    

    // Check that dmCmd has the proper size.
    if (dmCmd.size() != dm_totalElements) {
        std::cerr << "DM command vector size mismatch: expected " 
                  << dm_totalElements << ", got " << dmCmd.size() << std::endl;
        return;
    }
    
    // Signal that we are writing.
    dm_rtc.md->write = 1;
    
    // Ensure the write‐flag store happens before the memcpy
    std::atomic_thread_fence(std::memory_order_release);

    // Use memcpy to copy the new DM command values into the shared memory.
    // Ensure that the Eigen vector data is contiguous by using .data()
    std::memcpy(dm_rtc.array.D, dmCmd.data(), dm_totalElements * sizeof(double));
    
    // Update counters to notify consumers that data has been updated.
    dm_rtc.md->cnt0++;  
    dm_rtc.md->cnt1 = 0;
    
    // Post the semaphore once to signal that new data is available.
    int ret = ImageStreamIO_sempost(&dm_rtc, -1);
    if (ret != 0) {
        std::cerr << "Error posting semaphore: " << ret << std::endl;
    }
    
    // Clear the write flag.
    dm_rtc.md->write = 0;
}


// A helper function to print the dimension information of a matrix.
template <typename Derived>
void printMatrixDimensions(const std::string &name,
                             const Eigen::MatrixBase<Derived>& M) {
    std::cout << name << " dimensions: " 
              << M.rows() << " x " << M.cols() << std::endl;
}

// A helper function to print the size of a vector.
void printVectorSize(const std::string &name, const Eigen::VectorXd& v) {
    std::cout << name << " size: " << v.size() << std::endl;
}

// // The main RTC function
void rtc(){
    // Temporary variabiles that don't need initialisation
    // can go here.


    // std::cout << "Controller kp size: " << rtc_config.controller.kp.size() << std::endl;
    // std::cout << "Controller ki size: " << rtc_config.controller.ki.size() << std::endl;
    // std::cout << "Controller kd size: " << rtc_config.controller.kd.size() << std::endl;
    // std::cout << "Controller lower_limits size: " << rtc_config.controller.lower_limits.size() << std::endl;
    // std::cout << "Controller upper_limits size: " << rtc_config.controller.upper_limits.size() << std::endl;
    // std::cout << "Controller set_point size: " << rtc_config.controller.set_point.size() << std::endl;

    std::cout << "ctrl I2M_LO size: " << rtc_config.matrices.I2M_LO.size() << std::endl;
    std::cout << "ctrl I2M_HO size: " << rtc_config.matrices.I2M_HO.size() << std::endl;

    std::cout << "ctrl M2C_HO size: " << rtc_config.matrices.M2C_LO.size() << std::endl;
    std::cout << "ctrl M2C_HO size: " << rtc_config.matrices.M2C_HO.size() << std::endl;

    std::cout << "ctrl kp LO size: " << rtc_config.ctrl_LO.kp.size() << std::endl;

    std::cout << "ctrl kp size: " << rtc_config.ctrl_HO.kp.size() << std::endl;
    std::cout << "Controller ki size: " << rtc_config.ctrl_LO.ki.size() << std::endl;
    std::cout << "Controller kd size: " << rtc_config.ctrl_LO.kd.size() << std::endl;
    std::cout << "Controller lower_limits size: " << rtc_config.ctrl_LO.lower_limits.size() << std::endl;
    std::cout << "Controller upper_limits size: " << rtc_config.ctrl_LO.upper_limits.size() << std::endl;
    std::cout << "Controller set_point size: " << rtc_config.ctrl_LO.set_point.size() << std::endl;
    std::cout << "M2C_HO" << rtc_config.matrices.M2C_HO.size() << std::endl;

    std::cout << "secondary pixels size: " << rtc_config.pixels.secondary_pixels.size() << std::endl;
    std::cout << "exterior pixels size: " << rtc_config.pixels.exterior_pixels.size() << std::endl;

    std::cout << "secondary strehl matrix: " << rtc_config.matrices.I2rms_sec << std::endl;
    std::cout << "exterior strehl matrix: " << rtc_config.matrices.I2rms_ext << std::endl;
    
    std::cout << "szm: " << rtc_config.matrices.szm << std::endl;
    std::cout << "sza: " << rtc_config.matrices.sza << std::endl;
    std::cout << "szp: " << rtc_config.matrices.szp << std::endl;

    if (dm_rtc.md) {
        int dm_naxis = dm_rtc.md->naxis;
        int dm_width = dm_rtc.md->size[0];
        int dm_height = (dm_naxis > 1) ? dm_rtc.md->size[1] : 1;
        int dm_totalElements = dm_rtc.md->nelement; //  totalElements = width * height
        std::cout << "Shared memory size: " << dm_width << " x " << dm_height << " pixels ("
                << dm_totalElements << " elements)" << std::endl;
    } else {
        std::cerr << "Error: No metadata available in the dm shared memory image." << std::endl;
    }

    if (subarray.md) {
        int img_naxis = subarray.md->naxis;
        int img_width = subarray.md->size[0];
        int img_height = (img_naxis > 1) ? subarray.md->size[1] : 1;
        int img_totalElements = subarray.md->nelement; //  totalElements = width * height
        std::cout << "Shared memory size: " << img_width << " x " << img_height << " pixels ("
                << img_totalElements << " elements)" << std::endl;
    } else {
        std::cerr << "Error: No metadata available in the subarray shared memory image." << std::endl;
    }

    ////////////////////////////////////////////////////
    /// parameters below are calculated at run time (derived from rtc_config) so 
    // I made a method in the  bdr_rtc_config struct to init these..
    ////////
    //// Aspects to be re-computed everytime the configuration changes - init_rtc function? 
    // if (rtc_config.state.controller_type=="PID"){
    //     ////heree
    //     // PIDController ctrl_LO( rtc_config.controller );
    //     // PIDController ctrl_HO( rtc_config.controller ); // this will have to use different configs / lengths !  
    //     ctrl_LO = PIDController( rtc_config.ctrl_LO_config );
    //     ctrl_HO = PIDController( rtc_config.ctrl_HO_config ); // this will have to use different configs / lengths !  
    //     }
    // else{
    //     std::cout << "no ctrl match" << std::endl;
    // }


    // // some pre inits
    // Eigen::VectorXd img = Eigen::VectorXd::Zero(rtc_config.matrices.szp); // P must be defined appropriately.
    // Eigen::VectorXd zeroCmd = Eigen::VectorXd::Zero(rtc_config.matrices.sza);

    // ----------------------- IMPORTANT 
    // frames per second from rtc_config used for converting dark from adu/s -> adu 
    // we should actually read this from the current camera settings when calling the rtc! 
    // double fps = std::stof(rtc_config.cam.fps);
    // double gain = std::stof(rtc_config.cam.gain);
    // double scale = gain / fps;
    // // ALL I2M and reference intensities stored in config are in ADU/second/gain !!
    // Eigen::MatrixXd I2M_LO = scale * rtc_config.matrices.I2M_LO;
    // Eigen::MatrixXd I2M_HO = scale * rtc_config.matrices.I2M_HO;
    
    // Eigen::VectorXd N0_dm =  scale * rtc_config.reference_pupils.norm_pupil_dm;
    // Eigen::VectorXd I0_dm =  scale * rtc_config.reference_pupils.I0_dm;

    // // darks (generated in dcs/calibration_frames/gen_dark_bias_badpix.py ) are adu/s in the gain setting (not normalized by gain)
    // // I should review the darks and perhaps normalize by gain setting too! 
    // Eigen::VectorXd dark_dm = 1.0 / fps * rtc_config.reduction.dark_dm;



    // Strehl models
    // secondary pixel. Solarstein mask closer to UT size - this will be invalid on internal source 
    // int sec_idx = rtc_config.pixels.secondary_pixels(4); // .secondary_pixels defines 3x3 square around secondary - we only use the central one
    // double m_s = scale * rtc_config.matrices.I2rms_sec(0, 0); // intensity is normalized by fps and gain in model (/home/asg/Progs/repos/asgard-alignment/calibration/build_strehl_model.py)
    // double b_s = rtc_config.matrices.I2rms_sec(1, 1);
    
    //exterior pixels 
    //double m_e = rtc_config.matrices.I2rms_sec(0, 0);
    //double b_e = rtc_config.matrices.I2rms_sec(1, 1);




    // naughty actuator or modes 
    std::vector<int> naughty_list(140, 0);

    // time stuff 
    double current_time_ms;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);


    
    ///////////////////////////////////////


    /// DERIVED PARAMETERS FROM rtc_config
    // {
    //     std::scoped_lock lock(ctrl_mutex, telemetry_mutex); //C++17
    //     rtc_config.initDerivedParameters();
    // }
    //////////////////////////////////////


    const int expectedPixels = 32 * 32;

    // Pre‑compute totalPixels so we only do it once
    int nx = subarray.md->size[0];
    int ny = (subarray.md->naxis > 1) ? subarray.md->size[1] : 1;
    int totalPixels = nx * ny;
    if (totalPixels != expectedPixels) {
        throw std::runtime_error("RTC: shared‑memory geometry mismatch");
    }



    // pick an arbitrary preferred sem index (0 is fine)
    int semid = ImageStreamIO_getsemwaitindex(&subarray, /*preferred*/ 0);

    // LOOP SPEED (us)
    //std::chrono::microseconds loop_time( 1000 ); //static_cast<long long>(1.0/fps * 1e6) ); // microseconds - set speed of loop 
    constexpr auto loop_time = std::chrono::microseconds(1500); //  1 kHz
    auto next_tick = std::chrono::steady_clock::now();

    // before entering main loop, drain any stale posts:
    catch_up_with_sem(&subarray, semid);

    // either if we're open or closed loop we always keep a time consistent record of the telemetry members 
    while(servo_mode != SERVO_STOP){

        //std::cout << servo_mode_LO << std::endl;
        //std::cout << servo_mode_HO << std::endl;
        // Check if we need to pause the loop.
        if (pause_rtc.load()) {
            // Use a unique_lock with the mutex for the condition variable.
            // (A condition variable’s wait functions require a std::unique_lock<std::mutex> rather than a std::lock_guard<std::mutex> because the condition variable needs the flexibility to unlock and later re-lock the mutex during the wait. )
            std::unique_lock<std::mutex> lock(rtc_pause_mutex);
            // Wait until pause_rtc becomes false.
            // This lambda serves as a predicate.
            rtc_pause_cv.wait(lock, [](){ return !pause_rtc.load(); });
            // catch up on any missed semaphore posts exactly once after pause
            catch_up_with_sem(&subarray, semid);
            std::cout<< "caught up with semaphores amd resuming" << std::endl;
            //just_resumed = true;
        }

        // catch up on any missed semaphore posts exactly once after pause
        // if (just_resumed) {
        //     //ImageStreamIO_semflush(&subarray, -1); // maybe this but not clear
        //     catch_up_with_sem(&subarray, semid);
        //     just_resumed = false;
        //     std::cout<< "flushed semaphores amd resuming" << std::endl;
        // }

        start = std::chrono::steady_clock::now();


        //img =  getFrameFromSharedMemory(32*32); //32*32);
        
        //ImageStreamIO_semwait(&subarray, /*timeout=*/-1);

        ImageStreamIO_semwait(&subarray, semid);

        //  MAP & COPY into an Eigen vector
        //    (we know data is stored U16 so we cast to uint16_t*)
        uint16_t *raw = subarray.array.UI16;
        // Eigen::VectorXd img(totalPixels);
        // for (int i = 0; i < totalPixels; ++i) {
        //     img(i) = static_cast<double>(raw[i]);
        // }
        // a better way 
        // 1) Map the raw uint16_t buffer as an Eigen array:
        Eigen::Map<const Eigen::Array<uint16_t,Eigen::Dynamic,1>> rawArr(raw, totalPixels);

        // 2) Cast it to double—and store into a VectorXd:
        Eigen::VectorXd img = rawArr.cast<double>();

        //std::cout  << img.size() << std::endl;
        //uint64_t totalFramesWritten = subarray.md->cnt0;
        //uint64_t lastSliceIndex     = ImageStreamIO_readLastWroteIndex(&subarray);

        // // Print them
        // std::cout 
        //     << "Frame #" << totalFramesWritten 
        //     << " landed in buffer slot " << lastSliceIndex 
        //     << std::endl;
            

        // model of residual rms in DM units using secondary obstruction 
        dm_rms_est_1 = rtc_config.m_s_runtime * ( img[  rtc_config.sec_idx ] - rtc_config.reduction.dark[ rtc_config.sec_idx ] - rtc_config.reduction.bias[ rtc_config.sec_idx ] ) +  rtc_config.b_s_runtime;

        //std::cout << dm_rms_est_1 << std::endl;

        // go to dm space subtracting dark (ADU/s) and bias (ADU) there
        // should actually read the current fps rather then get it from config file
        img_dm = (rtc_config.matrices.I2A *  img)  -  rtc_config.dark_dm_runtime - rtc_config.reduction.bias_dm; //1 / fps * rtc_config.reduction.dark_dm;

        sig = (img_dm - rtc_config.I0_dm_runtime).cwiseQuotient(rtc_config.N0_dm_runtime); //(img_dm - rtc_config.reference_pupils.I0_dm).cwiseQuotient(rtc_config.reference_pupils.norm_pupil_dm);
        //sig = (rtc_config.matrices.I2A *  img - rtc_config.reference_pupils.I0_dm).cwiseQuotient(rtc_config.reference_pupils.norm_pupil_dm);

        e_LO =  rtc_config.I2M_LO_runtime * sig; //rtc_config.matrices.I2M_LO * sig ;

        e_HO =  rtc_config.I2M_HO_runtime * sig; //rtc_config.matrices.I2M_HO * sig ;


        // if auto mode change state based on signals


        // LO CALCULATIONS 
        if (servo_mode_LO == SERVO_CLOSE){
            if (!rtc_config.telem.LO_servo_mode.empty() && rtc_config.telem.LO_servo_mode.back() != SERVO_CLOSE) {
                std::cout << "LO IN CLOSED LOOP" << std::endl;

            }
            std::lock_guard<std::mutex> lock(ctrl_mutex);
            u_LO = rtc_config.ctrl_LO.process( e_LO );

            c_LO = rtc_config.matrices.M2C_LO * u_LO;
        
        }else if(servo_mode_LO == SERVO_OPEN){
            if (!rtc_config.telem.LO_servo_mode.empty() && rtc_config.telem.LO_servo_mode.back() != SERVO_OPEN) {
                std::cout << "reseting LO controller" << std::endl;
                //reset controllers
                rtc_config.ctrl_LO.reset();
            }
  
            u_LO = 0 * e_LO ;

            c_LO = rtc_config.matrices.M2C_LO * u_LO;
            
        }

        // HO CALCULATIONS 
        if (servo_mode_HO == SERVO_CLOSE){
            if (!rtc_config.telem.HO_servo_mode.empty() && rtc_config.telem.HO_servo_mode.back() != SERVO_CLOSE) {
                std::cout << "HO IN CLOSED LOOP" << std::endl;

            }

            u_HO = rtc_config.ctrl_HO.process( e_HO );
            
            c_HO = rtc_config.matrices.M2C_HO * u_HO;
            
            //std::cout << c_HO.cwiseAbs().maxCoeff() << std::endl; 
            //std::cout << rtc_config.matrices.M2C_HO.cwiseAbs().maxCoeff() << std::endl; 
            //std::cout << sig.cwiseAbs().maxCoeff() << std::endl; 

        }else if(servo_mode_HO == SERVO_OPEN){
            if (!rtc_config.telem.HO_servo_mode.empty() && rtc_config.telem.HO_servo_mode.back() != SERVO_OPEN) {
                std::cout << "reseting HO controller" << std::endl;
                //reset controllers
                rtc_config.ctrl_HO.reset();
            }
  

            u_HO = 0 * e_HO ;
          
            c_HO = rtc_config.matrices.M2C_HO * u_HO;
            
        }

        //std::cout << "e_LO size = " << e_LO.size() << std::endl;
        //std::cout << "e_HO size = " << e_HO.size() << std::endl;

        
        dmCmd = c_LO + c_HO;

        if (c_HO.cwiseAbs().maxCoeff() > rtc_config.limits.open_on_dm_limit) {
            // Find index of maximum absolute value.
            //int culprit = 0;
            //double maxVal = u_HO.cwiseAbs().maxCoeff(&culprit);
            //std::cout << "beam " << beam_id << " broke by act." << culprit << ", resetting" << std::endl;
            
            // Optionally, to reduce gain for this actuator by half, you might do:
            // rtc_config.ctrl_HO.ki(culprit) *= 0.5;
            
            // Set the DM command data to zeros.
            // Create a zero vector with the same length as u_HO.
            
            // Convert the vector to a 2D map via dm.cmd_2_map2D() and write to shared memory.
            //dm.set_data(dm.cmd_2_map2D(zeroCmd));

            // // zero dm
            // //updateDMSharedMemory( zeroCmd ) ;
            
            // // Reset the high-order controller.
            // rtc_config.ctrl_HO.reset();
            // rtc_config.ctrl_LO.reset();
            
            // // Increase the counter for this actuator.
            // naughty_list[culprit] += 1;
            
            // // If the controller's gain vector has fewer than 30 elements, stop the loop.
            // if (naughty_list[culprit] > 10) {
            //     servo_mode = SERVO_STOP;
            //     std::cout << "ENDING" << std::endl;
            // }
            
            // // If this actuator has been problematic more than 4 times, disable it by setting its gain to zero.
            // if (naughty_list[culprit] > 4) {
            //     std::cout << "Turn off naughty actuator " << culprit << std::endl;
            //     rtc_config.ctrl_HO.ki(culprit) = 0.0;
            // }
        }


        // ******************** UPDATE DM ******************************

        //updateDMSharedMemory( dmCmd ) ;

        // ******************** --------- ******************************

        // ************************************************************
        // Struc of ring buffers to keep history and offload to telemetry thread if requestec 
        if (true){
            std::lock_guard<std::mutex> lock(telemetry_mutex);
            // Get the current time as a double (microseconds)
            current_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch()
                                    ).count();
                                    
            // Append data to telemetry ring buffers.
            //std::cout << "time  " << current_time_ms  << std::endl;
            rtc_config.telem.timestamp.push_back(current_time_ms);

            rtc_config.telem.LO_servo_mode.push_back(servo_mode_LO);          // 'c_HO' is Eigen::VectorXd

            rtc_config.telem.HO_servo_mode.push_back(servo_mode_HO);          // 'c_HO' is Eigen::VectorXd

            //std::cout << "img size: " << img.size() << std::endl;
            rtc_config.telem.img.push_back(img);          // 'img' is Eigen::VectorXd
            //std::cout << "img_dm size: " << img_dm.size() << std::endl;
            rtc_config.telem.img_dm.push_back(img_dm);          // 'img' is Eigen::VectorXd
            //std::cout << ", sig size: " << sig.size() << std::endl;
            rtc_config.telem.signal.push_back(sig);       // 'sig' is Eigen::VectorXd

            //std::cout << ", sig size: " << sig.size() << std::endl;
            rtc_config.telem.e_LO.push_back(e_LO);          // 'e_LO' is Eigen::VectorXd

            //std::cout << ", sig size: " << sig.size() << std::endl;
            rtc_config.telem.u_LO.push_back(u_LO);          // 'u_LO' is Eigen::VectorXd

            //std::cout << ", sig size: " << sig.size() << std::endl;
            rtc_config.telem.e_HO.push_back(e_HO);          // 'e_HO' is Eigen::VectorXd

            //std::cout << ", sig size: " << sig.size() << std::endl;
            rtc_config.telem.u_HO.push_back(u_HO);          // 'u_HO' is Eigen::VectorXd

            //std::cout << ", sig size: " << c_LO.size() << std::endl;
            rtc_config.telem.c_LO.push_back(c_LO);          // 'c_LO' is Eigen::VectorXd

            //std::cout << ", c_HO size: " << c_HO.size() << std::endl;
            rtc_config.telem.c_HO.push_back(c_HO);          // 'c_HO' is Eigen::VectorXd

            // Increment the counter.
            rtc_config.telem.counter++;
        }

        // schedule next wakeup
        next_tick += loop_time;
        std::this_thread::sleep_until(next_tick);

        // detect overrun
        auto now = std::chrono::steady_clock::now();
        if (now > next_tick) {
            auto over = now - next_tick;
            std::cerr<<"Loop overran by "
                    << std::chrono::duration_cast<std::chrono::microseconds>(over).count()
                    <<" μs\n";
        }
        
        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // with telemetry and everything typically 220us loop without sleep (5kHz)
        std::cout << "duration microsec:  " << duration.count() << std::endl;

        /////////////////////// SLEEP ///////////////////////


        // if (duration < loop_time){
        //    std::this_thread::sleep_for(std::chrono::microseconds(loop_time - duration));
        // }


    }
    std::cout << "servo_mode changed to" << servo_mode << "...rtc stopped" << std::endl;
}












