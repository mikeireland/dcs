#include "baldr.h"
#include <chrono>
#include <iostream>
#include <ImageStreamIO.h>
#include <cstdint>  // for uint16_t


/*
dm_rtc.md->write = 1;
double* dotD = dm_rtc.array.D; // Shouldn't be a double!!! Write to dotD.
ImageStreamIO_sempost(dm_rtc, -1);
dm_rtc.md->write = 0;
*/

long unsigned int rtc_cnt=0;
unsigned int nerrors=0;

// Add local (to the RTC) variables here

// controllers
PIDController ctrl_LO ; 
PIDController ctrl_HO ; 

// intermediate products 
Eigen::VectorXd img;

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


// // The main RTC function
void rtc(){
    // Temporary variabiles that don't need initialisation
    // can go here.


    std::cout << "Controller kp size: " << rtc_config.controller.kp.size() << std::endl;
    std::cout << "Controller ki size: " << rtc_config.controller.ki.size() << std::endl;
    std::cout << "Controller kd size: " << rtc_config.controller.kd.size() << std::endl;
    std::cout << "Controller lower_limits size: " << rtc_config.controller.lower_limits.size() << std::endl;
    std::cout << "Controller upper_limits size: " << rtc_config.controller.upper_limits.size() << std::endl;
    std::cout << "Controller set_point size: " << rtc_config.controller.set_point.size() << std::endl;

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

    //PIDController ctrl( ); // rtc_config_list[0].controller );
    if (rtc_config.state.controller_type=="PID"){
        PIDController ctrl_LO( rtc_config.controller );
        PIDController ctrl_HO( rtc_config.controller ); // this will have to use different configs / lengths !  
        }
    else{
        std::cout << "no ctrl match" << std::endl;
    }

    Eigen::VectorXd img = Eigen::VectorXd::Zero(rtc_config.matrices.szp); // P must be defined appropriately.
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    while(servo_mode != SERVO_STOP){

        start = std::chrono::steady_clock::now();

        // need to subtract dark , bias norm
        img =  getFrameFromSharedMemory(1600);
        
        sig = ((rtc_config.matrices.I2A * img) - rtc_config.reference_pupils.I0_dm).cwiseQuotient(rtc_config.reference_pupils.norm_pupil_dm);
        
        //end = std::chrono::steady_clock::now();
        //duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        //std::cout << "Elapsed time sig: " << duration.count() << " us" << std::endl;
        
        //start = std::chrono::steady_clock::now();

        e_LO = rtc_config.matrices.I2M_LO * sig ;

        e_HO = rtc_config.matrices.I2M_HO * sig ;

        //end = std::chrono::steady_clock::now();
        //duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        //std::cout << "Elapsed time e_HO: " << duration.count() << " us" << std::endl;

        //start = std::chrono::steady_clock::now();

        u_LO = ctrl_LO.process( e_LO );

        u_HO = ctrl_HO.process( e_HO );

        //end = std::chrono::steady_clock::now();
        //duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        //std::cout << "Elapsed time u_HO: " << duration.count() << " us" << std::endl;

        c_LO = rtc_config.matrices.M2C_LO * u_LO;
        
        c_HO = rtc_config.matrices.M2C_HO * u_HO;

        dmCmd = c_LO + c_HO;
        //std::cout << "Elapsed time sig: " << c_LO  << std::endl;
        //<< e_HO.size() << "  ki size in PID: " << ctrl.ki.size() << "controller ki size used to configure PID" << rtc_config.controller.ki.size() << std::endl;
        updateDMSharedMemory( dmCmd ) ;

        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        std::cout << "wrote to dm. Full loop in " << duration.count() <<" us. wow." << std::endl;
    }


}
