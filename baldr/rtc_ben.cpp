#include <iostream>
#include <string>
#include "baldr.h"  // Include our shared header

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <beam_id> <phasemask>" << std::endl;
        return 1;
    }
    
    // Parse command-line arguments.
    int beam_id = 0;
    try {
        beam_id = std::stoi(argv[1]);
    } catch(const std::exception& e) {
        std::cerr << "Invalid beam_id provided. Must be an integer." << std::endl;
        return 1;
    }
    std::string phasemask = argv[2];
    
    // Build the configuration file path.
    std::string config_file = "/Users/bencb/Documents/ASGARD/cpp_tests/baldr_config_" 
                              + std::to_string(beam_id) + ".toml";
    
    // Use the readConfig function to parse the file.
    auto config = readConfig(config_file);
    
    // Now process the configuration
    // Global configuration: "baldr_pupils"
    if (auto baldr_node = config["baldr_pupils"]; baldr_node) {
        std::cout << "baldr_pupils key found." << std::endl;
    } else {
        std::cerr << "baldr_pupils key not found." << std::endl;
    }
    
    // Beam-specific configuration: "beam<beam_id>"
    std::string beamKey = "beam" + std::to_string(beam_id);
    auto beam_node = config[beamKey];
    if (!beam_node || !beam_node.is_table()) {
        std::cerr << "Beam configuration not found for key: " << beamKey << std::endl;
        return 1;
    }
    auto beam_table = *beam_node.as_table();
    
    //read I2A from the beam section.
    if (auto I2A_node = beam_table["I2A"]; I2A_node && I2A_node.is_array()) {
        Eigen::MatrixXd I2A = convertTomlArrayToEigenMatrix(*I2A_node.as_array());
        std::cout << "I2A matrix:" << std::endl << I2A << std::endl;
    }
    
    // (Additional processing for pupil_mask, control model, etc., goes here.)
    
    return 0;
}