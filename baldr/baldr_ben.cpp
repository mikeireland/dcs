#include <iostream>
#include <string>
#include <toml++/toml.h>
#include <Eigen/Dense>

// Convert a TOML array (assumed to be a 2D array of numbers) to an Eigen matrix.
Eigen::MatrixXd convertTomlArrayToEigenMatrix(const toml::array& arr) {
    size_t rows = arr.size();
    if (rows == 0) return Eigen::MatrixXd();
    
    // Assume each element is an array representing a row.
    const auto* firstRow = arr[0].as_array();
    size_t cols = firstRow ? firstRow->size() : 0;
    
    Eigen::MatrixXd mat(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        const auto* rowArr = arr[i].as_array();
        if (rowArr) {
            for (size_t j = 0; j < cols; ++j) {
                auto val_opt = rowArr->at(j).value<double>();
                mat(i, j) = val_opt ? *val_opt : 0.0;
            }
        }
    }
    return mat;
}

// Convert a TOML array (assumed to be a 2D array of booleans) to an Eigen boolean matrix.
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> convertTomlArrayToBoolMatrix(const toml::array& arr) {
    size_t rows = arr.size();
    if (rows == 0) return Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>();
    
    const auto* firstRow = arr[0].as_array();
    size_t cols = firstRow ? firstRow->size() : 0;
    
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mat(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        const auto* rowArr = arr[i].as_array();
        if (rowArr) {
            for (size_t j = 0; j < cols; ++j) {
                auto val_opt = rowArr->at(j).value<bool>();
                mat(i, j) = val_opt ? *val_opt : false;
            }
        }
    }
    return mat;
}


toml::table readConfig(const std::string &config_file) {
    try {
        auto config = toml::parse_file(config_file);
        return config;
    }
    catch(const toml::parse_error &err) {
        std::cerr << "TOML parse error: " << err.description() << std::endl;
        std::exit(1);
    }
}


// int main(int argc, char* argv[]) {
//     if (argc < 3) {
//         std::cerr << "Usage: " << argv[0] << " <beam_id> <phasemask>" << std::endl;
//         return 1;
//     }
    
//     // Parse command-line arguments.
//     int beam_id = 0;
//     try {
//         beam_id = std::stoi(argv[1]);
//     } catch(const std::exception& e) {
//         std::cerr << "Invalid beam_id provided. Must be an integer." << std::endl;
//         return 1;
//     }
//     std::string phasemask = argv[2];
    
//     // Build the config file path.
//     std::string config_file = "/Users/bencb/Documents/ASGARD/cpp_tests/baldr_config_" + std::to_string(beam_id) + ".toml";
    
//     try {
//         auto config = toml::parse_file(config_file);
        
//         // Global configuration: "baldr_pupils"
//         auto baldr_node = config["baldr_pupils"];
//         if(baldr_node) {
//             std::cout << "baldr_pupils key found." << std::endl;
//             // Process baldr_pupils as needed...
//         } else {
//             std::cerr << "baldr_pupils key not found." << std::endl;
//         }
        
//         // Beam-specific configuration: "beam<beam_id>"
//         std::string beamKey = "beam" + std::to_string(beam_id);
//         auto beam_node = config[beamKey];
//         if (!beam_node || !beam_node.is_table()) {
//             std::cerr << "Beam configuration not found for key: " << beamKey << std::endl;
//             return 1;
//         }
//         auto beam_table = *beam_node.as_table();
        
//         // Read I2A from the beam section.
//         if (auto I2A_node = beam_table["I2A"]; I2A_node && I2A_node.is_array()) {
//             Eigen::MatrixXd I2A = convertTomlArrayToEigenMatrix(*I2A_node.as_array());
//             std::cout << "I2A matrix:" << std::endl << I2A << std::endl;
//         }
        
//         // Read image pixel filters under "pupil_mask".
//         if (auto pupil_mask_node = beam_table["pupil_mask"]; pupil_mask_node && pupil_mask_node.is_table()) {
//             auto pupil_mask_table = *pupil_mask_node.as_table();
            
//             if (auto mask_node = pupil_mask_table["mask"]; mask_node && mask_node.is_array()) {
//                 auto pupil_mask = convertTomlArrayToBoolMatrix(*mask_node.as_array());
//                 std::cout << "pupil_mask:" << std::endl;
//                 for (int i = 0; i < pupil_mask.rows(); i++) {
//                     for (int j = 0; j < pupil_mask.cols(); j++) {
//                         std::cout << pupil_mask(i, j) << " ";
//                     }
//                     std::cout << std::endl;
//                 }
//             }
            
//             if (auto exter_node = pupil_mask_table["exterior"]; exter_node && exter_node.is_array()) {
//                 auto exter_mask = convertTomlArrayToBoolMatrix(*exter_node.as_array());
//                 std::cout << "exterior mask:" << std::endl;
//                 for (int i = 0; i < exter_mask.rows(); i++) {
//                     for (int j = 0; j < exter_mask.cols(); j++) {
//                         std::cout << exter_mask(i, j) << " ";
//                     }
//                     std::cout << std::endl;
//                 }
//             }
            
//             if (auto secon_node = pupil_mask_table["secondary"]; secon_node && secon_node.is_array()) {
//                 auto secon_mask = convertTomlArrayToBoolMatrix(*secon_node.as_array());
//                 std::cout << "secondary mask:" << std::endl;
//                 for (int i = 0; i < secon_mask.rows(); i++) {
//                     for (int j = 0; j < secon_mask.cols(); j++) {
//                         std::cout << secon_mask(i, j) << " ";
//                     }
//                     std::cout << std::endl;
//                 }
//             }
//         }
        
//         // Control model: under the beam section, keyed by the phase mask string.
//         if (auto phase_node = beam_table[phasemask]; phase_node && phase_node.is_table()) {
//             auto phase_table = *phase_node.as_table();
//             if (auto ctrl_model_node = phase_table["ctrl_model"]; ctrl_model_node && ctrl_model_node.is_table()) {
//                 auto ctrl_model_table = *ctrl_model_node.as_table();
                
//                 if (auto IM_node = ctrl_model_table["IM"]; IM_node && IM_node.is_array()) {
//                     Eigen::MatrixXd IM = convertTomlArrayToEigenMatrix(*IM_node.as_array());
//                     std::cout << "IM matrix:" << std::endl << IM << std::endl;
//                 }
//                 if (auto I2M_node = ctrl_model_table["I2M"]; I2M_node && I2M_node.is_array()) {
//                     Eigen::MatrixXd I2M = convertTomlArrayToEigenMatrix(*I2M_node.as_array());
//                     std::cout << "I2M matrix:" << std::endl << I2M << std::endl;
//                 }
//                 if (auto M2C_node = ctrl_model_table["M2C"]; M2C_node && M2C_node.is_array()) {
//                     Eigen::MatrixXd M2C = convertTomlArrayToEigenMatrix(*M2C_node.as_array());
//                     std::cout << "M2C matrix:" << std::endl << M2C << std::endl;
//                 }
//                 if (auto I0_node = ctrl_model_table["I0"]; I0_node && I0_node.is_array()) {
//                     Eigen::MatrixXd I0 = convertTomlArrayToEigenMatrix(*I0_node.as_array());
//                     std::cout << "I0 matrix:" << std::endl << I0 << std::endl;
//                 }
//                 if (auto N0_node = ctrl_model_table["N0"]; N0_node && N0_node.is_array()) {
//                     Eigen::MatrixXd N0 = convertTomlArrayToEigenMatrix(*N0_node.as_array());
//                     std::cout << "N0 matrix:" << std::endl << N0 << std::endl;
//                 }
//                 if (auto norm_pupil_node = ctrl_model_table["norm_pupil"]; norm_pupil_node && norm_pupil_node.is_array()) {
//                     Eigen::MatrixXd N0i = convertTomlArrayToEigenMatrix(*norm_pupil_node.as_array());
//                     std::cout << "N0i matrix:" << std::endl << N0i << std::endl;
//                 }
//                 if (auto innerFilt_node = ctrl_model_table["inner_pupil_filt"]; innerFilt_node && innerFilt_node.is_array()) {
//                     auto inside_edge_filt = convertTomlArrayToBoolMatrix(*innerFilt_node.as_array());
//                     std::cout << "inner_pupil_filt:" << std::endl;
//                     for (int i = 0; i < inside_edge_filt.rows(); i++) {
//                         for (int j = 0; j < inside_edge_filt.cols(); j++) {
//                             std::cout << inside_edge_filt(i, j) << " ";
//                         }
//                         std::cout << std::endl;
//                     }
//                 }
//                 // reduction products: camera_config, bad_pixel_mask, bias, dark.
//                 if (auto cam_config_node = ctrl_model_table["camera_config"]; cam_config_node && cam_config_node.is_table()) {
//                     std::cout << "camera_config is present." << std::endl;
//                     // Process further if needed.
//                 }
//                 if (auto bpm_node = ctrl_model_table["bad_pixel_mask"]; bpm_node && bpm_node.is_array()) {
//                     auto bad_pixel_mask = convertTomlArrayToBoolMatrix(*bpm_node.as_array());
//                     std::cout << "bad_pixel_mask:" << std::endl;
//                     for (int i = 0; i < bad_pixel_mask.rows(); i++) {
//                         for (int j = 0; j < bad_pixel_mask.cols(); j++) {
//                             std::cout << bad_pixel_mask(i, j) << " ";
//                         }
//                         std::cout << std::endl;
//                     }
//                 }
//                 if (auto bias_node = ctrl_model_table["bias"]; bias_node && bias_node.is_array()) {
//                     Eigen::MatrixXd bias = convertTomlArrayToEigenMatrix(*bias_node.as_array());
//                     std::cout << "bias matrix:" << std::endl << bias << std::endl;
//                 }
//                 if (auto dark_node = ctrl_model_table["dark"]; dark_node && dark_node.is_array()) {
//                     Eigen::MatrixXd dark = convertTomlArrayToEigenMatrix(*dark_node.as_array());
//                     std::cout << "dark matrix:" << std::endl << dark << std::endl;
//                 }
//             }
//         }
        
//         // Strehl model configuration.
//         if (auto strehl_node = beam_table["strehl_model"]; strehl_node && strehl_node.is_table()) {
//             auto strehl_table = *strehl_node.as_table();
//             if (auto sec_node = strehl_table["secondary"]; sec_node && sec_node.is_array()) {
//                 Eigen::MatrixXd strehl_coe_sec = convertTomlArrayToEigenMatrix(*sec_node.as_array());
//                 std::cout << "strehl_coe_sec matrix:" << std::endl << strehl_coe_sec << std::endl;
//             }
//             if (auto ext_node = strehl_table["exterior"]; ext_node && ext_node.is_array()) {
//                 Eigen::MatrixXd strehl_coe_ext = convertTomlArrayToEigenMatrix(*ext_node.as_array());
//                 std::cout << "strehl_coe_ext matrix:" << std::endl << strehl_coe_ext << std::endl;
//             }
//         }
//     }
//     catch(const toml::parse_error& err) {
//         std::cerr << "TOML parse error: " << err.description() << std::endl;
//         return 1;
//     }
    
//     return 0;
// }