#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <toml++/toml.h>
#include <Eigen/Dense>
#include <string>


// Converts a TOML array (2D array of numbers) to an Eigen matrix.
Eigen::MatrixXd convertTomlArrayToEigenMatrix(const toml::array& arr);

// Converts a TOML array (2D array of booleans) to an Eigen boolean matrix.
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> convertTomlArrayToBoolMatrix(const toml::array& arr);

// Reads and parses a TOML configuration file from the given path.
toml::table readConfig(const std::string &config_file);
struct PIDConfig {
    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    Eigen::VectorXd lower_limit;
    Eigen::VectorXd upper_limit;
    Eigen::VectorXd setpoint;
};

//----- PID Controller declaration -----

class PIDController {
public:
    // Gains and limits.
    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    Eigen::VectorXd lower_limit;
    Eigen::VectorXd upper_limit;
    Eigen::VectorXd setpoint;
    
    // Internal state.
    Eigen::VectorXd output;
    Eigen::VectorXd integrals;
    Eigen::VectorXd prev_errors;
    
    std::string ctrl_type; // e.g., "PID"
    
    // Parameterized constructor.
    PIDController(const Eigen::VectorXd& kp_in,
                  const Eigen::VectorXd& ki_in,
                  const Eigen::VectorXd& kd_in,
                  const Eigen::VectorXd& lower_limit_in,
                  const Eigen::VectorXd& upper_limit_in,
                  const Eigen::VectorXd& setpoint_in);
    
    // New constructor accepting a PIDConfig struct.
    PIDController(const PIDConfig& config);
    
    // Default constructor with one-element defaults.
    PIDController();
    
    // Process measured input and compute the controller output.
    Eigen::VectorXd process(const Eigen::VectorXd& measured);
    
    // Set all gains to zero.
    void set_all_gains_to_zero();
    
    // Reset internal state (integrals, previous errors, and output).
    void reset();
};

#endif // BALDR_H