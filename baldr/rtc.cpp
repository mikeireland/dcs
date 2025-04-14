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
PIDController ctrl_LO ; 
PIDController ctrl_HO ; 

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

    std::cout << "ctrl kp size: " << ctrl_LO.kp.size() << std::endl;

    std::cout << "ctrl kp size: " << ctrl_LO.kp.size() << std::endl;
    std::cout << "Controller ki size: " << ctrl_LO.ki.size() << std::endl;
    std::cout << "Controller kd size: " << ctrl_LO.kd.size() << std::endl;
    std::cout << "Controller lower_limits size: " << ctrl_LO.lower_limits.size() << std::endl;
    std::cout << "Controller upper_limits size: " << ctrl_LO.upper_limits.size() << std::endl;
    std::cout << "Controller set_point size: " << ctrl_LO.set_point.size() << std::endl;
    std::cout << "M2C_HO" << rtc_config.matrices.M2C_HO.size() << std::endl;

    
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


    if (rtc_config.state.controller_type=="PID"){
        ////heree
        // PIDController ctrl_LO( rtc_config.controller );
        // PIDController ctrl_HO( rtc_config.controller ); // this will have to use different configs / lengths !  
        PIDController ctrl_LO( rtc_config.ctrl_HO_config );
        PIDController ctrl_HO( rtc_config.ctrl_LO_config ); // this will have to use different configs / lengths !  
        }
    else{
        std::cout << "no ctrl match" << std::endl;
    }

    // frames per second from rtc_config used for converting dark from adu/s -> adu 
    float fps = std::stof(rtc_config.cam.fps);

    Eigen::VectorXd img = Eigen::VectorXd::Zero(rtc_config.matrices.szp); // P must be defined appropriately.

    // zero command 
    Eigen::VectorXd zeroCmd = Eigen::VectorXd::Zero(rtc_config.matrices.sza);

    // naughty actuator or modes 
    std::vector<int> naughty_list(140, 0);

    // time stuff 
    double current_time_ms;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);


    // either if we're open or closed loop we always keep a time consistent record of the telemetry members 
    while(servo_mode != SERVO_STOP){

        //std::cout << servo_mode_LO << std::endl;
        //std::cout << servo_mode_HO << std::endl;

        start = std::chrono::steady_clock::now();

        // need to subtract dark , bias norm
        img =  getFrameFromSharedMemory(32*32);
        
        // go to dm space subtracting dark (ADU/s) and bias (ADU) there
        // should actually read the current fps rather then get it from config file
        img_dm = (rtc_config.matrices.I2A *  img)  - 1/fps * rtc_config.reduction.dark_dm - rtc_config.reduction.bias_dm;

        sig = (img_dm - rtc_config.reference_pupils.I0_dm).cwiseQuotient(rtc_config.reference_pupils.norm_pupil_dm);
        //sig = (rtc_config.matrices.I2A *  img - rtc_config.reference_pupils.I0_dm).cwiseQuotient(rtc_config.reference_pupils.norm_pupil_dm);

        e_LO = rtc_config.matrices.I2M_LO * sig ;

        e_HO = rtc_config.matrices.I2M_HO * sig ;


        // if auto mode change state based on signals


        // LO CALCULATIONS 
        if (servo_mode_LO == SERVO_CLOSE){

            u_LO = ctrl_LO.process( e_LO );

            c_LO = rtc_config.matrices.M2C_LO * u_LO;
        
        }else if(servo_mode_LO == SERVO_OPEN){
            if (!rtc_config.telem.LO_servo_mode.empty() && rtc_config.telem.LO_servo_mode.back() != SERVO_OPEN) {
                std::cout << "reseting HO controller" << std::endl;
                //reset controllers
                ctrl_LO.reset();
            }
  
            u_LO = 0 * e_LO ;

            c_LO = rtc_config.matrices.M2C_LO * u_LO;
            
        }

        // HO CALCULATIONS 
        if (servo_mode_HO == SERVO_CLOSE){

            u_HO = ctrl_HO.process( e_HO );
            
            c_HO = rtc_config.matrices.M2C_HO * u_HO;


        //write_telemetry_to_json(rtc_config.telem, "/home/asg/Music/telemetry.json");
        }else if(servo_mode_HO == SERVO_OPEN){
            if (!rtc_config.telem.HO_servo_mode.empty() && rtc_config.telem.HO_servo_mode.back() != SERVO_OPEN) {
                std::cout << "reseting HO controller" << std::endl;
                //reset controllers
                ctrl_HO.reset();
            }
  

            u_HO = 0 * e_HO ;
          
            c_HO = rtc_config.matrices.M2C_HO * u_HO;
            
        }


        dmCmd = c_LO + c_HO;

        if (dmCmd.cwiseAbs().maxCoeff() > rtc_config.limits.open_on_dm_limit) {
            // Find index of maximum absolute value.
            int culprit = 0;
            //double maxVal = u_HO.cwiseAbs().maxCoeff(&culprit);
            std::cout << "beam " << beam_id << " broke by act." << culprit << ", resetting" << std::endl;
            
            // Optionally, to reduce gain for this actuator by half, you might do:
            // ctrl_HO.ki(culprit) *= 0.5;
            
            // Set the DM command data to zeros.
            // Create a zero vector with the same length as u_HO.
            
            // Convert the vector to a 2D map via dm.cmd_2_map2D() and write to shared memory.
            //dm.set_data(dm.cmd_2_map2D(zeroCmd));

            // zero dm
            updateDMSharedMemory( zeroCmd ) ;
            
            // Reset the high-order controller.
            ctrl_HO.reset();
            ctrl_LO.reset();
            
            // Increase the counter for this actuator.
            naughty_list[culprit] += 1;
            
            // If the controller's gain vector has fewer than 30 elements, stop the loop.
            if (naughty_list[culprit] > 10) {
                servo_mode = SERVO_STOP;
                std::cout << "ENDING" << std::endl;
            }
            
            // If this actuator has been problematic more than 4 times, disable it by setting its gain to zero.
            if (naughty_list[culprit] > 4) {
                std::cout << "Turn off naughty actuator " << culprit << std::endl;
                ctrl_HO.ki(culprit) = 0.0;
            }
        }



        //updateDMSharedMemory( dmCmd ) ;


        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);


        // ************************************************************
        // Struc of ring buffers to keep history and offload to telemetry thread if requestec 
        if (true){
            std::lock_guard<std::mutex> lock(telemetry_mutex);
            // Get the current time as a double (for example, in milliseconds)
            current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch()
                                    ).count();
                                    
            // Append data to telemetry ring buffers.
            // (If you're updating telemetry from multiple threads, you should lock a mutex around these updates.)
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
    }
    std::cout << "servo_mode changed to" << servo_mode << "...rtc stopped" << std::endl;
}












