#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>
#include <stdexcept>
#include <utility>

// Abstract base class for controllers
class Controller {
public:
    virtual ~Controller() = default;

    // Process the measurement and update the state, returning the control output
    virtual Eigen::VectorXd process(const Eigen::VectorXd& measured) = 0;

    // Reset the internal states of the controller
    virtual void reset() = 0;

    // Set all gains to zero
    virtual void set_all_gains_to_zero() = 0;

    // Get the current state of the controller (error, integral, derivative, etc.)
    virtual Eigen::VectorXd get_state() const = 0;

    // Get the control output
    virtual Eigen::VectorXd get_output() const = 0;

    // Return the controller type (e.g., "PID", "Leaky Integrator", "Kalman")
    virtual std::string get_type() const = 0;

    // Set the current setpoint (desired target value)
    virtual void set_setpoint(const Eigen::VectorXd& new_setpoint) = 0;

    // Get the current setpoint (desired target value)
    virtual Eigen::VectorXd get_setpoint() const = 0;

    // Set a parameter (gain, matrix, or any other internal attribute) dynamically
    virtual void set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) = 0;

    // Get a parameter's value dynamically
    virtual std::variant<Eigen::VectorXd, Eigen::MatrixXd> get_parameter(const std::string& param_name) const = 0;

    // List all parameters and their descriptions
    virtual std::vector<std::pair<std::string, std::string>> list_parameters() const = 0;

protected:
    Eigen::VectorXd state;    // General state vector (error, integral, derivative, etc.)
    Eigen::VectorXd output;   // Control output
    Eigen::VectorXd setpoint; // Desired target (setpoint)

    // Store parameters in a key-value map, where key = parameter name, value = parameter value
    std::unordered_map<std::string, std::variant<Eigen::VectorXd, Eigen::MatrixXd>> parameters;

    // Store descriptions of the parameters for the list_parameters method
    std::unordered_map<std::string, std::string> parameter_descriptions;
};

// PIDController (inherits Controller)
class PIDController : public Controller {
public:
    PIDController(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd, double dt);
    ~PIDController() override = default;

    // Process the measurement and update the state, returning the control output
    Eigen::VectorXd process(const Eigen::VectorXd& measured) override;

    // Reset the internal states of the controller
    void reset() override;

    // Set all gains to zero
    void set_all_gains_to_zero() override;

    // Get the current state (error, integral, derivative)
    Eigen::VectorXd get_state() const override;

    // Get the control output
    Eigen::VectorXd get_output() const override;

    // Return the controller type
    std::string get_type() const override;

    // Set the current setpoint (desired target value)
    void set_setpoint(const Eigen::VectorXd& new_setpoint) override;

    // Get the current setpoint (desired target value)
    Eigen::VectorXd get_setpoint() const override;

    // Set a parameter (gain or matrix) dynamically
    void set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) override;

    // Get a parameter's value dynamically
    std::variant<Eigen::VectorXd, Eigen::MatrixXd> get_parameter(const std::string& param_name) const override;

    // List all parameters and their descriptions
    std::vector<std::pair<std::string, std::string>> list_parameters() const override;

private:
    Eigen::VectorXd Kp, Ki, Kd;  // Gains
    double dt;                   // Time step
    Eigen::VectorXd prev_errors; // Previous errors
    Eigen::VectorXd integrals;   // Integral state
    // // dont think i need this: double prev_error;  // Previous error for derivative calculation

    std::string ctrl_type = "PID";  // Default value set to "PID"
};

// LeakyIntegratorController (inherits Controller)
class LeakyIntegratorController : public Controller {
public:
    LeakyIntegratorController(const Eigen::VectorXd& K, const Eigen::VectorXd& alpha, double dt);
    ~LeakyIntegratorController() override = default;

    // Process the measurement and update the state, returning the control output
    Eigen::VectorXd process(const Eigen::VectorXd& measured) override;

    // Reset the internal states of the controller
    void reset() override;

    // Set all gains to zero
    void set_all_gains_to_zero() override;

    // Get the current state (integral)
    Eigen::VectorXd get_state() const override;

    // Get the control output
    Eigen::VectorXd get_output() const override;

    // Return the controller type
    std::string get_type() const override;

    // Set the current setpoint (desired target value)
    void set_setpoint(const Eigen::VectorXd& new_setpoint) override;

    // Get the current setpoint (desired target value)
    Eigen::VectorXd get_setpoint() const override;

    // Set a parameter (gain or matrix) dynamically
    void set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) override;

    // Get a parameter's value dynamically
    std::variant<Eigen::VectorXd, Eigen::MatrixXd> get_parameter(const std::string& param_name) const override;

    // List all parameters and their descriptions
    std::vector<std::pair<std::string, std::string>> list_parameters() const override;

private:
    Eigen::VectorXd K, alpha;  // Gain and leakage factor
    double dt;                // Time step
    Eigen::VectorXd integral;  // Integral state

    Eigen::VectorXd prev_error; // Previous error for leakage computation

    std::string ctrl_type = "LEAKY";  // Default value set to "PID"
};

// KalmanController (inherits Controller)
class KalmanController : public Controller {
public:
    KalmanController(int state_size, int measurement_size, double dt);
    ~KalmanController() override = default;

    // Process the measurement and update the state, returning the control output
    Eigen::VectorXd process(const Eigen::VectorXd& measured) override;

    // Reset the internal states of the controller
    void reset() override;

    // Set all gains to zero (Kalman typically does not have "gains")
    void set_all_gains_to_zero() override;

    // Get the current state (state estimate)
    Eigen::VectorXd get_state() const override;

    // Get the control output
    Eigen::VectorXd get_output() const override;

    // Return the controller type
    std::string get_type() const override;

    // Set the current setpoint (desired target value)
    void set_setpoint(const Eigen::VectorXd& new_setpoint) override;

    // Get the current setpoint (desired target value)
    Eigen::VectorXd get_setpoint() const override;

    // Set a parameter (gain or matrix) dynamically
    void set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) override;

    // Get a parameter's value dynamically
    std::variant<Eigen::VectorXd, Eigen::MatrixXd> get_parameter(const std::string& param_name) const override;

    // List all parameters and their descriptions
    std::vector<std::pair<std::string, std::string>> list_parameters() const override;

private:
    int state_size, measurement_size;
    double dt;

    Eigen::MatrixXd A, B, H, P, Q, R; // Kalman Filter matrices
    Eigen::VectorXd x, u, z;          // State, control input, and measurement
    
    std::string ctrl_type = "KALMAN";  // Default value set to "KALMAN"
};

#endif // CONTROLLER_H
