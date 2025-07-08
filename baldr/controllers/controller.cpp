#include "controller.h"
#include <iostream>

// PIDController Implementation
PIDController::PIDController(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd, double dt)
    : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt) {
    output = Eigen::VectorXd::Zero(Kp.size());
    prev_errors = Eigen::VectorXd::Zero(Kp.size());
    integrals = Eigen::VectorXd::Zero(Kp.size());
}

Eigen::VectorXd PIDController::process(const Eigen::VectorXd& measured) {
    Eigen::VectorXd error = setpoint - measured;

    // Proportional term
    Eigen::VectorXd P_term = Kp.cwiseProduct(error);

    // Integral term
    integrals += error * dt;
    Eigen::VectorXd I_term = Ki.cwiseProduct(integrals);

    // Derivative term
    Eigen::VectorXd D_term = Kd.cwiseProduct(error - prev_errors) / dt;

    // Combine all terms to form the output
    output = P_term + I_term + D_term;

    // Update previous errors
    prev_errors = error;

    return output;
}

void PIDController::reset() {
    prev_errors.setZero();
    integrals.setZero();
    output.setZero();
}

void PIDController::set_all_gains_to_zero() {
    Kp.setZero();
    Ki.setZero();
    Kd.setZero();
}

Eigen::VectorXd PIDController::get_state() const {
    return integrals;  // Example: return integral as the state (could be extended)
}

Eigen::VectorXd PIDController::get_output() const {
    return output;
}

std::string PIDController::get_type() const {
    return "PID";
}

void PIDController::set_setpoint(const Eigen::VectorXd& new_setpoint) {
    setpoint = new_setpoint;
}

Eigen::VectorXd PIDController::get_setpoint() const {
    return setpoint;
}

void PIDController::set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) {
    if (param_name == "Kp") {
        Kp = std::get<Eigen::VectorXd>(value);
    } else if (param_name == "Ki") {
        Ki = std::get<Eigen::VectorXd>(value);
    } else if (param_name == "Kd") {
        Kd = std::get<Eigen::VectorXd>(value);
    } else {
        throw std::invalid_argument("Unknown parameter name");
    }
}

std::variant<Eigen::VectorXd, Eigen::MatrixXd> PIDController::get_parameter(const std::string& param_name) const {
    if (param_name == "Kp") {
        return Kp;
    } else if (param_name == "Ki") {
        return Ki;
    } else if (param_name == "Kd") {
        return Kd;
    } else {
        throw std::invalid_argument("Unknown parameter name");
    }
}

std::vector<std::pair<std::string, std::string>> PIDController::list_parameters() const {
    return {
        {"Kp", "Proportional gain vector"},
        {"Ki", "Integral gain vector"},
        {"Kd", "Derivative gain vector"}
    };
}


// LeakyIntegratorController Implementation
LeakyIntegratorController::LeakyIntegratorController(const Eigen::VectorXd& K, const Eigen::VectorXd& alpha, double dt)
    : K(K), alpha(alpha), dt(dt) {
    integral.setZero(K.size());
    prev_error.setZero(K.size());
}

Eigen::VectorXd LeakyIntegratorController::process(const Eigen::VectorXd& measured) {
    Eigen::VectorXd error = setpoint - measured;
    integral = integral + (K.cwiseProduct(error) * dt);
    integral = integral - (alpha.cwiseProduct(integral) * dt); // Apply leakage

    prev_error = error;
    return integral;
}

void LeakyIntegratorController::reset() {
    integral.setZero();
    prev_error.setZero();
}

void LeakyIntegratorController::set_all_gains_to_zero() {
    K.setZero();
    alpha.setZero();
}

Eigen::VectorXd LeakyIntegratorController::get_state() const {
    return integral;
}

Eigen::VectorXd LeakyIntegratorController::get_output() const {
    return integral;
}

std::string LeakyIntegratorController::get_type() const {
    return "Leaky Integrator";
}

void LeakyIntegratorController::set_setpoint(const Eigen::VectorXd& new_setpoint) {
    setpoint = new_setpoint;
}

Eigen::VectorXd LeakyIntegratorController::get_setpoint() const {
    return setpoint;
}

void LeakyIntegratorController::set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) {
    if (param_name == "K") {
        K = std::get<Eigen::VectorXd>(value);
    } else if (param_name == "alpha") {
        alpha = std::get<Eigen::VectorXd>(value);
    } else {
        throw std::invalid_argument("Unknown parameter name");
    }
}

std::variant<Eigen::VectorXd, Eigen::MatrixXd> LeakyIntegratorController::get_parameter(const std::string& param_name) const {
    if (param_name == "K") {
        return K;
    } else if (param_name == "alpha") {
        return alpha;
    } else {
        throw std::invalid_argument("Unknown parameter name");
    }
}

std::vector<std::pair<std::string, std::string>> LeakyIntegratorController::list_parameters() const {
    return {
        {"K", "Gain vector for leaky integrator"},
        {"alpha", "Leakage factor vector"}
    };
}


// KalmanController Implementation
KalmanController::KalmanController(int state_size, int measurement_size, double dt)
    : state_size(state_size), measurement_size(measurement_size), dt(dt) {
    A = Eigen::MatrixXd::Identity(state_size, state_size);
    B = Eigen::MatrixXd::Zero(state_size, measurement_size);
    H = Eigen::MatrixXd::Identity(measurement_size, state_size);
    Q = Eigen::MatrixXd::Identity(state_size, state_size);
    R = Eigen::MatrixXd::Identity(measurement_size, measurement_size);
    P = Eigen::MatrixXd::Identity(state_size, state_size);
    x = Eigen::VectorXd::Zero(state_size);
    u = Eigen::VectorXd::Zero(measurement_size);
    z = Eigen::VectorXd::Zero(measurement_size);
}

Eigen::VectorXd KalmanController::process(const Eigen::VectorXd& measured) {
    // Prediction step
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;

    // Update step
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    z = measured;
    x = x + K * (z - H * x);
    P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * H) * P;

    return x;
}

void KalmanController::reset() {
    x.setZero();
    P.setIdentity();
}

void KalmanController::set_all_gains_to_zero() {
    Q.setZero();
    R.setZero();
}

Eigen::VectorXd KalmanController::get_state() const {
    return x;
}

Eigen::VectorXd KalmanController::get_output() const {
    return x;
}

std::string KalmanController::get_type() const {
    return "Kalman Filter";
}

void KalmanController::set_setpoint(const Eigen::VectorXd& new_setpoint) {
    // Kalman controller doesn't typically use setpoints, but this can be added for consistency
}

Eigen::VectorXd KalmanController::get_setpoint() const {
    return Eigen::VectorXd::Zero(state_size);  // Kalman doesn't use setpoints in a traditional sense
}

void KalmanController::set_parameter(const std::string& param_name, const std::variant<Eigen::VectorXd, Eigen::MatrixXd>& value) {
    if (param_name == "A") {
        A = std::get<Eigen::MatrixXd>(value);
    } else if (param_name == "B") {
        B = std::get<Eigen::MatrixXd>(value);
    } else if (param_name == "H") {
        H = std::get<Eigen::MatrixXd>(value);
    } else if (param_name == "Q") {
        Q = std::get<Eigen::MatrixXd>(value);
    } else if (param_name == "R") {
        R = std::get<Eigen::MatrixXd>(value);
    } else {
        throw std::invalid_argument("Unknown parameter name");
    }
}

std::variant<Eigen::VectorXd, Eigen::MatrixXd> KalmanController::get_parameter(const std::string& param_name) const {
    if (param_name == "A") {
        return A;
    } else if (param_name == "B") {
        return B;
    } else if (param_name == "H") {
        return H;
    } else if (param_name == "Q") {
        return Q;
    } else if (param_name == "R") {
        return R;
    } else {
        throw std::invalid_argument("Unknown parameter name");
    }
}

std::vector<std::pair<std::string, std::string>> KalmanController::list_parameters() const {
    return {
        {"A", "State transition matrix"},
        {"B", "Control matrix"},
        {"H", "Measurement matrix"},
        {"Q", "Process noise covariance"},
        {"R", "Measurement noise covariance"}
    };
}
