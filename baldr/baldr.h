#ifndef BALDR_H
#define BALDR_H

#include <toml++/toml.h>
#include <Eigen/Dense>
#include <string>

// Converts a TOML array (2D array of numbers) to an Eigen matrix.
Eigen::MatrixXd convertTomlArrayToEigenMatrix(const toml::array& arr);

// Converts a TOML array (2D array of booleans) to an Eigen boolean matrix.
Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> convertTomlArrayToBoolMatrix(const toml::array& arr);

// Reads and parses a TOML configuration file from the given path.
toml::table readConfig(const std::string &config_file);

#endif // BALDR_H