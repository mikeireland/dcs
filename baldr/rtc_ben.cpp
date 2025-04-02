#include <iostream>
#include <string>
#include "baldr_ben.h"  // Contains our readConfig, convertTomlArrayToEigenMatrix, etc.

//clang++ -std=c++17 -I/opt/tomlplusplus/include -I/opt/homebrew/include/eigen3 rtc_ben.cpp baldr_ben.cpp -o rtc_test

Eigen::MatrixXd dm_cmd_2_map2D(const Eigen::VectorXd& cmd) {
    int n = cmd.size();
    int rows = 2;  // For example, assume the DM map has 2 rows.
    int cols = n / rows; // Assume n is divisible by 2.
    Eigen::MatrixXd map(rows, cols);
    // Fill the matrix row-by-row.
    for (int i = 0; i < n; ++i) {
        map(i / cols, i % cols) = cmd(i);
    }
    return map;
}

// Simulate sending a DM command by printing it.
void dm_set_data(const Eigen::MatrixXd& dcmd) {
    std::cout << "Setting DM data:\n" << dcmd << "\n";
}


Eigen::VectorXd get_image(int n) {
    // Return a random vector with values between 0 and 1.
    return Eigen::VectorXd::Random(n).array() * 0.5 + 0.5;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <beam_id> <phasemask>" << std::endl;
        return 1;
    }
    
    int beam_id = std::stoi(argv[1]);
    std::string phasemask = argv[2];
    
    // Build the configuration file path.
    std::string config_file = "/Users/bencb/Documents/ASGARD/dcs/baldr/baldr_config_" 
                              + std::to_string(beam_id) + ".toml";
    
    // Read the full TOML configuration.
    auto config = readConfig(config_file);
    
    // 1. Process the global configuration "baldr_pupils"
    if (auto baldr_node = config["baldr_pupils"]; baldr_node) {
        std::cout << "Global configuration 'baldr_pupils' found." << std::endl;
        // You can further process this section if needed.
    } else {
        std::cerr << "'baldr_pupils' key not found." << std::endl;
    }
    
    // 2. Access the beam-specific configuration, e.g. "beam2"
    std::string beamKey = "beam" + std::to_string(beam_id);
    auto beam_node = config[beamKey];
    if (!beam_node || !beam_node.is_table()) {
        std::cerr << "Beam configuration not found for key: " << beamKey << std::endl;
        return 1;
    }
    auto beam_table = *beam_node.as_table();
    
    // Get this from the ctrl node
    // // 3. Read I2A and perform an operation on it.
    // if (auto I2A_node = beam_table["I2A"]; I2A_node && I2A_node.is_array()) {
    //     Eigen::MatrixXd I2A = convertTomlArrayToEigenMatrix(*I2A_node.as_array());
    //     std::cout << "I2A matrix:" << std::endl << I2A << std::endl;
    //     // For example, you might want to compute its transpose:
    //     std::cout << "I2A transpose:" << std::endl << I2A.transpose() << std::endl;
    // }
    
    Eigen::MatrixXd I2M, M2C, I2A, I0, bias, dark, N0i, strehl_coe_ext, strehl_coe_sec;
    Eigen::VectorXd N0dm, I0dm, bias_dm, dark_dm ,badpixmap ;
    double fps = 0.0;
    double gain = 0.0;

    // 4. Read image pixel filters under "pupil_mask".
    if (auto pupil_mask_node = beam_table["pupil_mask"]; pupil_mask_node && pupil_mask_node.is_table()) {
        auto pupil_mask_table = *pupil_mask_node.as_table();
        
        // Process "mask"
        if (auto mask_node = pupil_mask_table["mask"]; mask_node && mask_node.is_array()) {
            auto pupil_mask = convertTomlArrayToBoolMatrix(*mask_node.as_array());
            std::cout << "pupil_mask:" << std::endl;
            for (int i = 0; i < pupil_mask.rows(); i++) {
                for (int j = 0; j < pupil_mask.cols(); j++) {
                    std::cout << pupil_mask(i, j) << " ";
                }
                std::cout << std::endl;
            }
        }
        
        // Process "exterior" and "secondary" similarly...
    }
    
    // 5. Access the control model section for the given phase mask.
    if (auto phase_node = beam_table[phasemask]; phase_node && phase_node.is_table()) {
        auto phase_table = *phase_node.as_table();

        // Inside the ctrl_model processing block (in rtc.cpp):
        if (auto ctrl_model_node = phase_table["ctrl_model"]; ctrl_model_node && ctrl_model_node.is_table()) {
            auto ctrl_model_table = *ctrl_model_node.as_table();
            
            std::cout << ctrl_model_table << std::endl;
            
            // Process matricies
            // if (auto IM_node = ctrl_model_table["IM"]; IM_node && IM_node.is_array()) {
            //     Eigen::MatrixXd IM = convertTomlArrayToEigenMatrix(*IM_node.as_array());
            //     std::cout << "IM matrix:" << std::endl << IM << std::endl;
            // }
            if (auto I2M_node = ctrl_model_table["I2M"]; I2M_node && I2M_node.is_array()) {
                I2M = convertTomlArrayToEigenMatrix(*I2M_node.as_array());
                std::cout << "I2M matrix:" << std::endl << I2M << std::endl;
            }
            if (auto M2C_node = ctrl_model_table["M2C"]; M2C_node && M2C_node.is_array()) {
                M2C = convertTomlArrayToEigenMatrix(*M2C_node.as_array());
                std::cout << "M2C matrix:" << std::endl << M2C << std::endl;
            }

            if (auto I2A_node = ctrl_model_table["I2A"]; I2A_node && I2A_node.is_array()) {
                I2A = convertTomlArrayToEigenMatrix(*I2A_node.as_array());
                std::cout << "Control model I2A matrix:" << std::endl << I2A << std::endl;
            } else {
                std::cerr << "Control model I2A not found." << std::endl;
                return 1;
            }
            

            if (auto norm_pupil_node = ctrl_model_table["norm_pupil"]; norm_pupil_node && norm_pupil_node.is_array()) {
                N0i = convertTomlArrayToEigenMatrix(*norm_pupil_node.as_array());
                std::cout << "norm_pupil (N0i) matrix:" << std::endl << N0i << std::endl;
            }
            if (auto I0_node = ctrl_model_table["I0"]; I0_node && I0_node.is_array()) {
                I0 = convertTomlArrayToEigenMatrix(*I0_node.as_array());
                std::cout << "I0 matrix:" << std::endl << I0 << std::endl;
            }
            if (auto bias_node = ctrl_model_table["bias"]; bias_node && bias_node.is_array()) {
                bias = convertTomlArrayToEigenMatrix(*bias_node.as_array());
                std::cout << "bias matrix:" << std::endl << bias << std::endl;
            }
            if (auto dark_node = ctrl_model_table["dark"]; dark_node && dark_node.is_array()) {
                dark = convertTomlArrayToEigenMatrix(*dark_node.as_array());
                std::cout << "dark matrix:" << std::endl << dark << std::endl;
            }
            
            // For bad_pixel_mask, get the boolean matrix.
            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> bpm_bool;
            if (auto bpm_node = ctrl_model_table["bad_pixel_mask"]; bpm_node && bpm_node.is_array()) {
                bpm_bool = convertTomlArrayToBoolMatrix(*bpm_node.as_array());
                std::cout << "bad_pixel_mask:" << std::endl;
                for (int i = 0; i < bpm_bool.rows(); ++i) {
                    for (int j = 0; j < bpm_bool.cols(); ++j) {
                        std::cout << bpm_bool(i, j) << " ";
                    }
                    std::cout << std::endl;
                }
            }
            
            // Reshape each 2D matrix into a vector.
            // Eigen::Map converts the matrix data into a vector view.
            Eigen::VectorXd N0i_vec = Eigen::Map<Eigen::VectorXd>(N0i.data(), N0i.size());
            Eigen::VectorXd I0_vec   = Eigen::Map<Eigen::VectorXd>(I0.data(), I0.size());
            Eigen::VectorXd bias_vec = Eigen::Map<Eigen::VectorXd>(bias.data(), bias.size());
            Eigen::VectorXd dark_vec = Eigen::Map<Eigen::VectorXd>(dark.data(), dark.size());
            
            // For the boolean matrix, build a double vector (1.0 for true, 0.0 for false).
            Eigen::VectorXd bpm_vec(bpm_bool.size());
            for (int i = 0; i < bpm_bool.rows(); ++i) {
                for (int j = 0; j < bpm_bool.cols(); ++j) {
                    bpm_vec(i * bpm_bool.cols() + j) = bpm_bool(i, j) ? 1.0 : 0.0;
                }
            }
            
            // Perform the matrix multiplications.
            N0dm = I2A * N0i_vec;
            I0dm = I2A * I0_vec;
            bias_dm = I2A * bias_vec;
            dark_dm = I2A * dark_vec;
            badpixmap = I2A * bpm_vec;
            
            // Output the results.
            std::cout << "N0dm:" << std::endl << N0dm << std::endl;
            std::cout << "I0dm:" << std::endl << I0dm << std::endl;
            std::cout << "bias_dm:" << std::endl << bias_dm << std::endl;
            std::cout << "dark_dm:" << std::endl << dark_dm << std::endl;
            std::cout << "badpixmap:" << std::endl << badpixmap << std::endl;


            /// this is configuring fps and gain from IM settings - should prob use the current settings..
            if (auto cam_config_node = ctrl_model_table["camera_config"]; cam_config_node && cam_config_node.is_table()) {
                auto cam_config_table = *cam_config_node.as_table();
                if (auto fps_node = cam_config_table["fps"]; fps_node) {
                    fps = fps_node.value<double>().value_or(0.0);
                }
                if (auto gain_node = cam_config_table["gain"]; gain_node) {
                    gain = gain_node.value<double>().value_or(0.0);
                }
                std::cout << "Camera config: fps = " << fps << ", gain = " << gain << std::endl;
            } else {
                std::cerr << "camera_config not found in ctrl_model." << std::endl;
            }
            
        }


    }
    
    // 6. Process the strehl model configuration.
    if (auto strehl_node = beam_table["strehl_model"]; strehl_node && strehl_node.is_table()) {
        auto strehl_table = *strehl_node.as_table();
        if (auto sec_node = strehl_table["secondary"]; sec_node && sec_node.is_array()) {
            strehl_coe_sec = convertTomlArrayToEigenMatrix(*sec_node.as_array());
            std::cout << "strehl_coe_sec matrix:" << std::endl << strehl_coe_sec << std::endl;
        }
        if (auto ext_node = strehl_table["exterior"]; ext_node && ext_node.is_array()) {
            strehl_coe_ext = convertTomlArrayToEigenMatrix(*ext_node.as_array());
            std::cout << "strehl_coe_ext matrix:" << std::endl << strehl_coe_ext << std::endl;
        }
    }


    // ----- Example: Using the PID Controller with a configuration struct -----
    // Create a PIDConfig with sample values.
    // Eventually this should be a commander struc that can be edited while RTC is runnning!!
    PIDConfig pidConfig;
    pidConfig.kp = Eigen::VectorXd(2);
    pidConfig.ki = Eigen::VectorXd(2);
    pidConfig.kd = Eigen::VectorXd(2);
    pidConfig.lower_limit = Eigen::VectorXd(2);
    pidConfig.upper_limit = Eigen::VectorXd(2);
    pidConfig.setpoint = Eigen::VectorXd(2);
    
    pidConfig.kp << 1.0, 1.0;
    pidConfig.ki << 0.1, 0.1;
    pidConfig.kd << 0.01, 0.01;
    pidConfig.lower_limit << -10.0, -10.0;
    pidConfig.upper_limit << 10.0, 10.0;
    pidConfig.setpoint << 5.0, 5.0;
    
    // Create the PIDController using the PIDConfig struct.
    PIDController pid(pidConfig);
    
    // Simulate measured process values.
    Eigen::VectorXd measured(2);
    measured << 3.0, 4.0;
    
    // Process and output the PID results.
    Eigen::VectorXd pid_output = pid.process(measured);
    std::cout << "PID output: " << pid_output.transpose() << std::endl;
    


    // // define our image length based on I2M matrix in toml config file
    //std::cout << "I2M matrix:" << std::endl << I2M << std::endl;
    int image_length = I2M.rows();

    // Main RTC control loop (simulate 10 iterations)
    for (int iter = 0; iter < 10; ++iter) {
        std::cout << "----- Iteration " << iter+1 << " -----" << std::endl;
        
        // 1. Acquire raw intensity from camera (simulate with random numbers).
        Eigen::VectorXd i = get_image(image_length);
        
        // // 2. Model of turbulence (in DM units).
        // // For simulation, take the 5th element of the image (if exists) as representative.
        // double strehl_signal = 0.0;
        // if (i.size() > 4) {
        //     strehl_signal = i(4) - bias_sec - (1.0/fps * dark_sec);
        // }
        // double dm_rms_est = strehl_coe_sec(0) * strehl_signal + strehl_coe_sec(1);
        // std::cout << "strehl_signal: " << strehl_signal << ", dm_rms_est: " << dm_rms_est << std::endl;
        
        // 3. Project the image into DM space.
        // Flatten the image vector is already 1D; multiply I2A with i.
        Eigen::VectorXd idm = I2A * i - (gain/fps) * dark_dm - bias_dm;
        
        // 4. Compute the ADU normalized pupil signal pre prohected to DM actuator registered pixels.
        // s = (idm - I0dm) / N0dm, element-wise division.
        Eigen::VectorXd s = (idm - I0dm).cwiseQuotient(N0dm);
        
        // 5. Compute error: e = D * s.
        Eigen::VectorXd e = I2M * s;
        
        // 6. Run the PID controller.
        Eigen::VectorXd u = pid.process(e);
        
        // 7. Safety check.
        if (u.cwiseAbs().maxCoeff() > 100000000000000) {
            std::cout << "Safety triggered: max(|u|) > 0.4. Resetting DM and PID controller." << std::endl;
            dm_set_data(Eigen::MatrixXd::Zero(2, 1));  // Set DM data to zeros.
            pid.reset();
            // Optionally, continue or break out.
        }
        
        // 8. Remove piston by subtracting the mean of u.
        u.array() -= u.mean();
        
        // 9. Reconstruction: compute DM command.
        Eigen::MatrixXd dcmd = -1.0 * dm_cmd_2_map2D(u);
        
        // 10. Send DM command.
        dm_set_data(dcmd);
        
        // Sleep to simulate cycle time.
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
 
    return 0;
}





// # project reference intensities to DM (quicker for division & subtraction)
// N0dm = (I2A @ N0i.reshape(-1)) # these are already reduced #- dark_dm - bias_dm
// I0dm = (I2A @ I0.reshape(-1)) # these are already reduced  #- dark_dm - bias_dm
// bias_dm = I2A @ bias.reshape(-1)
// dark_dm = I2A @ dark.reshape(-1)
// badpixmap = I2A @ bad_pixel_mask.astype(int).reshape(-1)


// for it in range(args.number_of_iterations):
    

//     # raw intensity 
//     i = c.get_image() # fill this with randon numbers

//     # model of the turbulence (in DM units)
//     strehl_signal = (i[secon_mask.astype(bool)][4] - bias_sec - (1/ fps * dark_sec) )
//     dm_rms_est = strehl_coe_sec[0] *  strehl_signal + strehl_coe_sec[1]

//     # go to dm space subtracting dark (ADU/s) and bias (ADU) there
//     idm = (I2A @ i.reshape(-1))  - 1/fps * dark_dm - bias_dm

//     # adu normalized pupil signal 
//     s =  ( idm - I0dm ) / (N0dm)   # 

//     # error
//     e = D @ s 

//     # ctrl 
//     u = ctrl_HO.process( e )
//     # if it > close_after:
//     #     u = ctrl_HO.process( e )
//     # else:
//     #     u = 0 * e 

    
//     # safety
//     if np.max( abs( u ) ) > 0.4:
//         print("broke, reseting")
//         dm.set_data( np.zeros( len(u)) )
//         #dm.activate_calibrated_flat()
//         ctrl_HO.reset()
//         #break

    
//     u -= np.mean( u ) # Forcefully remove piston! 
//     # reconstruction
//     dcmd = -1 *  dm.cmd_2_map2D(u) 
//     t1 = time.time()

//     dm.set_data( dcmd )













// /////////// TESTING SPEED DIFFERENCE OF EIGEN AND BASIC C MATRIX MULTIPLCATION, TO CONVINCE MIKE TO USE EIGEN
// constexpr int N = 200;         // Matrix dimensions: N x N.
// constexpr int iterations = 100; // Number of multiplications.

// // --- Setup: Create two random matrices using Eigen ---
// Eigen::MatrixXd A = Eigen::MatrixXd::Random(N, N);
// Eigen::MatrixXd B = Eigen::MatrixXd::Random(N, N);
// Eigen::MatrixXd C; // For Eigen result.

// // --- Prepare plain C-style arrays for multiplication ---
// double* a = new double[N * N];
// double* b = new double[N * N];
// double* c = new double[N * N];

// // Copy data from Eigen matrices into plain arrays.
// for (int i = 0; i < N * N; ++i) {
//     a[i] = A.data()[i];
//     b[i] = B.data()[i];
//     c[i] = 0.0;
// }

// // --- Timing the plain C-style nested loop multiplication ---
// volatile double dummy = 0.0; // Volatile accumulator to prevent optimization.
// auto startC = std::chrono::high_resolution_clock::now();
// for (int iter = 0; iter < iterations; ++iter) {
//     for (int i = 0; i < N; ++i) {
//         for (int j = 0; j < N; ++j) {
//             double sum = 0.0;
//             for (int k = 0; k < N; ++k) {
//                 sum += a[i * N + k] * b[k * N + j];
//             }
//             c[i * N + j] = sum;
//             dummy = sum; // Use the result so it isn't optimized away (I found the timer reported 0ms consistently - its because the optimizer didn't bother doing this because it was never used!  )
//         }
//     }
// }

// auto endC = std::chrono::high_resolution_clock::now();
// auto durationC = std::chrono::duration_cast<std::chrono::milliseconds>(endC - startC).count();
// std::cout << "Base C multiplication took " << durationC << " ms" << std::endl;
// std::cout << "Dummy value: " << dummy << std::endl; // Print to force usage.

// // --- Timing Eigen's built-in matrix multiplication ---
// auto startEigen = std::chrono::high_resolution_clock::now();
// for (int iter = 0; iter < iterations; ++iter) {
//     C = A * B;
// }
// auto endEigen = std::chrono::high_resolution_clock::now();
// auto durationEigen = std::chrono::duration_cast<std::chrono::milliseconds>(endEigen - startEigen).count();
// std::cout << "Eigen multiplication took " << durationEigen << " ms" << std::endl;

// // Clean up
// delete[] a;
// delete[] b;
// delete[] c;
