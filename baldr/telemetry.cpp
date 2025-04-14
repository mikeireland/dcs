// telemetry.cpp
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "baldr.h"   // Contains rtc_config and bdr_telem definition
#include <mutex>

// long unsigned int telemetry_cnt=0;
// // Add local (to telemetry) variables here

// // Initialise variables assocated with the RTC.
// void initialise_telemetry(){
//     // Add initialisation code here
// }

using json = nlohmann::json;

// Assume you have a mutex declared globally to protect access to telemetry:
extern std::mutex telemetry_mutex;

// A helper function to convert telemetry data into JSON.
// For example, here we assume each telemetry field is a ring buffer of Eigen::VectorXd,
// and for the timestamp we have a ring buffer of doubles.
json telemetry_to_json(const bdr_telem &telemetry) {
    json j;
    j["counter"] = telemetry.counter;

    // Convert timestamps:
    json ts = json::array();
    for (const double t : telemetry.timestamp) {
        ts.push_back(t);
    }
    j["timestamps"] = ts;

    j["LO_servo_mode"] = telemetry.LO_servo_mode;
    j["HO_servo_mode"] = telemetry.HO_servo_mode;

    // For each telemetry field that is a ring buffer of Eigen::VectorXd, convert it.
    auto eigen_vec_to_json = [](const boost::circular_buffer<Eigen::VectorXd>& buff) -> json {
        json arr = json::array();
        for (const auto &v : buff) {
            // Convert Eigen::VectorXd to std::vector<double>
            std::vector<double> vec(v.data(), v.data() + v.size());
            arr.push_back(vec);
        }
        return arr;
    };

    j["img"] = eigen_vec_to_json(telemetry.img);
    j["img_dm"] = eigen_vec_to_json(telemetry.img_dm);
    j["signal"] = eigen_vec_to_json(telemetry.signal);
    j["e_LO"] = eigen_vec_to_json(telemetry.e_LO);
    j["u_LO"] = eigen_vec_to_json(telemetry.u_LO);
    j["e_HO"] = eigen_vec_to_json(telemetry.e_HO);
    j["u_HO"] = eigen_vec_to_json(telemetry.u_HO);
    j["c_LO"] = eigen_vec_to_json(telemetry.c_LO);
    j["c_HO"] = eigen_vec_to_json(telemetry.c_HO);



    return j;
}



// The main RTC function
void telemetry(){
    // Temporary variabiles that don't need initialisation
    // can go here.
    //initialise_telemetry();

    while (servo_mode != SERVO_STOP) {  // keep_going is a global flag indicating the system is running


        // Sleep for a fixed interval (say, 1 second) before processing telemetry
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const std::string filename = "telemetry.json";
        
        if (rtc_config.state.take_telemetry){

            auto now = std::chrono::system_clock::now();                
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::localtime(&t);

            std::ostringstream oss;
            oss << "/home/asg/Music/telemetry_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".json";
            std::string filename = oss.str();

            bdr_telem currentTelem;
            {
                // Lock telemetry mutex, copy the telemetry object.
                std::lock_guard<std::mutex> lock(telemetry_mutex);
                currentTelem = rtc_config.telem;  // Ensure your telemetry struct supports copy.
            }
            
            // Convert the telemetry data to JSON.
            json j = telemetry_to_json(rtc_config.telem);
            
            // Write to file.
            std::ofstream ofs(filename);
            if (ofs.is_open()) {
                ofs << j.dump(4);
                ofs.close();
                std::cout << "Telemetry written to " << filename << std::endl;
            } else {
                std::cerr << "Error opening " << filename << " for writing." << std::endl;
            }
            std::cout << "Done writing " << std::endl;
            // set back to false
            rtc_config.state.take_telemetry = 0;
        }
    }
}
