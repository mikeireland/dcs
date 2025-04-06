#include "baldr.h"
#include <chrono>
long unsigned int rtc_cnt=0;
unsigned int nerrors=0;
// Add local (to the RTC) variables here



PIDController ctrl ; //rtc_config_list[0].controller);


Eigen::VectorXd img;
Eigen::VectorXd e_HO;
Eigen::VectorXd u_HO;
Eigen::VectorXd c_HO;
Eigen::VectorXd e_TT;
Eigen::VectorXd u_TT;
Eigen::VectorXd c_TT;
Eigen::VectorXd sig;
// Initialise RTC configurations for multiple beams.
// beam_ids: a vector of beam IDs.
// phaseKeys: a vector of corresponding phase mask strings.

// void initialise_rtc(){
//     int beam_id = 2;
//     std::string config_file = "baldr_config_" + std::to_string(beam_id) + ".toml";
//     std::string beamKey = "beam" + std::to_string(beam_id);
//     std::string phaseKey = "H3";
//     bdr_rtc_config config = readBDRConfig(config_file, beamKey, phaseKey);

// }

void initialise_rtc( ) {
    Eigen::VectorXd img; 
    Eigen::VectorXd e_H0;
    Eigen::VectorXd u_HO;
    Eigen::VectorXd e_TT;
    Eigen::VectorXd u_TT;
    //init controller
    //PIDController ctrl( );// rtc_config_list[0].controller );
    // if (rtc_config_list[0].state.controller_type=="PID"){
    //     PIDController ctrl( rtc_config_list[0].controller );
    //     }
    // else{
    //     std::cout << "no ctrl match" << std::endl;
    // }
}


// The main RTC function
void rtc(){
    // Temporary variabiles that don't need initialisation
    // can go here.

    
    // try {
    //     // Call initialise_rtc with the vectors.
    //     //initialise_rtc();
    //     //initialise_rtc(beams, phaseKeys);
    //     std::cout << "RTC configuration initialized for " << " beams." << std::endl;
    // } catch (const std::exception& ex) {
    //     std::cerr << "Error initializing RTC configuration: " << ex.what() << std::endl;

    // }

    // std::cout << "asd" << std::endl;

    initialise_rtc();

    std::cout << "Controller kp size: " << rtc_config_list[0].controller.kp.size() << std::endl;
    std::cout << "Controller ki size: " << rtc_config_list[0].controller.ki.size() << std::endl;
    std::cout << "Controller kd size: " << rtc_config_list[0].controller.kd.size() << std::endl;
    std::cout << "Controller lower_limits size: " << rtc_config_list[0].controller.lower_limits.size() << std::endl;
    std::cout << "Controller upper_limits size: " << rtc_config_list[0].controller.upper_limits.size() << std::endl;
    std::cout << "Controller set_point size: " << rtc_config_list[0].controller.set_point.size() << std::endl;

    std::cout << "ctrl kp size: " << ctrl.kp.size() << std::endl;
    std::cout << "Controller ki size: " << ctrl.ki.size() << std::endl;
    std::cout << "Controller kd size: " << ctrl.kd.size() << std::endl;
    std::cout << "Controller lower_limits size: " << ctrl.lower_limits.size() << std::endl;
    std::cout << "Controller upper_limits size: " << ctrl.upper_limits.size() << std::endl;
    std::cout << "Controller set_point size: " << ctrl.set_point.size() << std::endl;


    // For now, just use beam 0. simulatie getting image
    Eigen::VectorXd img = Eigen::VectorXd::Zero(rtc_config_list[0].matrices.szp); // P must be defined appropriately.
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    while(servo_mode != SERVO_STOP){


        start = std::chrono::steady_clock::now();

        // need to add dark , bias norm
        //sig = rtc_config_list[0].matrices.I2A * ((img - rtc_config_list[0].reference_pupils.I0).cwiseQuotient(rtc_config_list[0].reference_pupils.N0));
        sig = ((rtc_config_list[0].matrices.I2A * img) - rtc_config_list[0].reference_pupils.I0_dm).cwiseQuotient(rtc_config_list[0].reference_pupils.norm_pupil_dm);
        
        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Elapsed time sig: " << duration.count() << " us" << std::endl;
        

        start = std::chrono::steady_clock::now();

        e_HO = rtc_config_list[0].matrices.I2M * sig ;

        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Elapsed time e_HO: " << duration.count() << " us" << std::endl;

        start = std::chrono::steady_clock::now();
        
        u_HO = ctrl.process( e_HO );
        
        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Elapsed time u_HO: " << duration.count() << " us" << std::endl;

        c_HO = rtc_config_list[0].matrices.M2C_HO * u_HO;



        //<< e_HO.size() << "  ki size in PID: " << ctrl.ki.size() << "controller ki size used to configure PID" << rtc_config_list[0].controller.ki.size() << std::endl;
    }
    // rtc_cnt = subarray.md->cnt0;
    // while(servo_mode != SERVO_STOP){
    //     if (subarray.md->cnt0 != rtc_cnt) {
    //         if (subarray.md->cnt0 > rtc_cnt+1) {
    //             std::cout << "Missed frame" << subarray.md->cnt0 << rtc_cnt << std::endl;
    //             nerrors++;
    //         }
    //     // Add RTC code here
    //     rtc_cnt++;
    //     }
    //     // !!! Something better than sleep would be good
    //     usleep(10);
    // }
}
