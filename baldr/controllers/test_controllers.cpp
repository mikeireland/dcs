#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "controller.h"
#include <Eigen/Dense>

// from this directory (on mac)
//clang++ -std=c++17 -Wall -I/opt/homebrew/opt/eigen/include/eigen3 -I../../catch2 test_controllers.cpp controller.cpp -o test_controllers

TEST_CASE("PIDController_1 reset and gain change", "[PID]") {
    Eigen::VectorXd Kp = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::VectorXd Ki = Eigen::VectorXd::Constant(2, 0.1);
    Eigen::VectorXd Kd = Eigen::VectorXd::Constant(2, 0.05);
    double dt = 0.1;



    PIDController_1 pid(Kp, Ki, Kd, dt);
    pid.set_setpoint(Eigen::VectorXd::Zero(2));

    Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::VectorXd out1 = pid.process(meas);
    REQUIRE(out1[0] < 0);

    pid.reset();
    Eigen::VectorXd out2 = pid.process(meas);
    REQUIRE(out2[0] < 0);
    REQUIRE(out2[0] == Approx(out1[0])); // should behave same after reset

    pid.set_all_gains_to_zero();
    Eigen::VectorXd out3 = pid.process(meas);
    REQUIRE(out3.norm() == Approx(0.0));
}

TEST_CASE("PIDController_1 parameter interface", "[PID]") {
    Eigen::VectorXd Kp = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::VectorXd Ki = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(2);
    double dt = 0.1;

    PIDController_1 pid(Kp, Ki, Kd, dt);

    Eigen::VectorXd newKp = Eigen::VectorXd::Constant(2, 0.5);
    pid.set_parameter("Kp", newKp);

    auto value = std::get<Eigen::VectorXd>(pid.get_parameter("Kp"));
    REQUIRE(value[0] == Approx(0.5));
}

TEST_CASE("LeakyIntegratorController basic functionality", "[Leaky]") {
    Eigen::VectorXd K = Eigen::VectorXd::Constant(2, 0.5);
    Eigen::VectorXd alpha = Eigen::VectorXd::Constant(2, 0.1);
    double dt = 0.1;

    LeakyIntegratorController li(K, alpha, dt);
    li.set_setpoint(Eigen::VectorXd::Zero(2));

    Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::VectorXd out = li.process(meas);

    REQUIRE(out.size() == 2);
    REQUIRE(out[0] < 0);
    REQUIRE(out[1] < 0);
}

TEST_CASE("LeakyIntegratorController basic behavior", "[Leaky]") {
    Eigen::VectorXd K = Eigen::VectorXd::Constant(2, 0.5);
    Eigen::VectorXd alpha = Eigen::VectorXd::Constant(2, 0.1);
    double dt = 0.1;

    LeakyIntegratorController ctrl(K, alpha, dt);
    ctrl.set_setpoint(Eigen::VectorXd::Zero(2));

    SECTION("Produces nonzero output for constant error") {
        Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
        Eigen::VectorXd out = ctrl.process(meas);
        REQUIRE(out.norm() > 0.0);
    }

    SECTION("Reset clears internal state") {
        Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
        ctrl.process(meas);  // accumulate some state
        ctrl.reset();        // now clear it

        Eigen::VectorXd out = ctrl.process(meas);  // should match fresh output
        REQUIRE(out.norm() > 0.0);  // not zero, but fresh
    }

    SECTION("Zero gains without reset does NOT guarantee zero output") {
        Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
        ctrl.process(meas);  // accumulate state

        ctrl.set_all_gains_to_zero();  // only sets K and alpha = 0
        Eigen::VectorXd out = ctrl.process(meas);
        REQUIRE(out.norm() > 0.0);  // previous state still present
    }

    SECTION("Zero gains AND reset leads to zero output") {
        Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
        ctrl.process(meas);  // accumulate state

        ctrl.set_all_gains_to_zero();
        ctrl.reset();  // clear state too

        Eigen::VectorXd out = ctrl.process(meas);
        REQUIRE(out.norm() == Approx(0.0));
    }

    SECTION("Parameter interface round-trip for K") {
        Eigen::VectorXd newK = Eigen::VectorXd::Constant(2, 0.25);
        ctrl.set_parameter("K", newK);
        auto roundtripK = std::get<Eigen::VectorXd>(ctrl.get_parameter("K"));
        REQUIRE(roundtripK.isApprox(newK));
    }

    SECTION("Parameter interface round-trip for alpha") {
        Eigen::VectorXd newAlpha = Eigen::VectorXd::Constant(2, 0.75);
        ctrl.set_parameter("alpha", newAlpha);
        auto roundtripAlpha = std::get<Eigen::VectorXd>(ctrl.get_parameter("alpha"));
        REQUIRE(roundtripAlpha.isApprox(newAlpha));
    }
}


TEST_CASE("KalmanController interface and reset", "[Kalman]") {
    KalmanController kc(2, 2, 0.1);
    kc.set_setpoint(Eigen::VectorXd::Zero(2));

    Eigen::VectorXd meas = Eigen::VectorXd::Constant(2, 1.0);
    Eigen::VectorXd out1 = kc.process(meas);
    REQUIRE(out1.size() == 2);

    kc.reset();
    Eigen::VectorXd out2 = kc.process(meas);
    REQUIRE(out2.size() == 2);

    kc.set_all_gains_to_zero();  // No-op but should not crash

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    kc.set_parameter("Q", Q);

    auto Q_result = std::get<Eigen::MatrixXd>(kc.get_parameter("Q"));
    REQUIRE(Q_result == Q);
}

// #include "controller.h"
// #include <iostream>
// #include <Eigen/Dense>


// // // clang++ -std=c++17 -Wall -I/opt/homebrew/opt/eigen/include/eigen3 test_controllers.cpp controller.cpp -o test_controllers


// #include "controller.h"
// #include <iostream>
// #include <Eigen/Dense>

// void test_pid_2d() {
//     std::cout << "--- Testing PIDController_1 (2D) ---\n";
//     Eigen::VectorXd Kp = Eigen::VectorXd::Constant(2, 1.0);
//     Eigen::VectorXd Ki = Eigen::VectorXd::Constant(2, 0.1);
//     Eigen::VectorXd Kd = Eigen::VectorXd::Constant(2, 0.05);
//     double dt = 0.1;

//     PIDController_1 pid(Kp, Ki, Kd, dt);
//     pid.set_setpoint(Eigen::VectorXd::Zero(2));

//     for (int i = 0; i < 10; ++i) {
//         Eigen::VectorXd measured = Eigen::VectorXd::Constant(2, 1.0);
//         Eigen::VectorXd output = pid.process(measured);
//         std::cout << "Step " << i << ", output = " << output.transpose() << "\n";
//     }
// }

// void test_leaky_2d() {
//     std::cout << "--- Testing LeakyIntegratorController (2D) ---\n";
//     Eigen::VectorXd K = Eigen::VectorXd::Constant(2, 0.5);
//     Eigen::VectorXd alpha = Eigen::VectorXd::Constant(2, 0.1);
//     double dt = 0.1;

//     LeakyIntegratorController li(K, alpha, dt);
//     li.set_setpoint(Eigen::VectorXd::Zero(2));

//     for (int i = 0; i < 10; ++i) {
//         Eigen::VectorXd measured = Eigen::VectorXd::Constant(2, 1.0);
//         Eigen::VectorXd output = li.process(measured);
//         std::cout << "Step " << i << ", output = " << output.transpose() << "\n";
//     }
// }

// void test_kalman_2d() {
//     std::cout << "--- Testing KalmanController (2D stub) ---\n";
//     KalmanController kc(2, 2, 0.1);
//     kc.set_setpoint(Eigen::VectorXd::Zero(2));

//     for (int i = 0; i < 10; ++i) {
//         Eigen::VectorXd measured = Eigen::VectorXd::Constant(2, 1.0);
//         Eigen::VectorXd output = kc.process(measured);
//         std::cout << "Step " << i << ", output = " << output.transpose() << "\n";
//     }
// }

// int main() {
//     test_pid_2d();
//     test_leaky_2d();
//     test_kalman_2d();
//     return 0;
// }

// // void test_pid() {
// //     std::cout << "--- Testing PIDController_1 ---\n";
// //     Eigen::VectorXd Kp = Eigen::VectorXd::Constant(1, 1.0);
// //     Eigen::VectorXd Ki = Eigen::VectorXd::Constant(1, 0.1);
// //     Eigen::VectorXd Kd = Eigen::VectorXd::Constant(1, 0.01);
// //     double dt = 0.1;

// //     PIDController_1 pid(Kp, Ki, Kd, dt);
// //     pid.set_setpoint(Eigen::VectorXd::Zero(1));

// //     for (int i = 0; i < 10; ++i) {
// //         Eigen::VectorXd meas = Eigen::VectorXd::Constant(1, 1.0);  // constant error
// //         Eigen::VectorXd out = pid.process(meas);
// //         std::cout << "Step " << i << ", output = " << out.transpose() << "\n";
// //     }
// // }

// // void test_leaky() {
// //     std::cout << "--- Testing LeakyIntegratorController ---\n";
// //     Eigen::VectorXd K = Eigen::VectorXd::Constant(1, 0.5);
// //     Eigen::VectorXd alpha = Eigen::VectorXd::Constant(1, 0.1);
// //     double dt = 0.1;

// //     LeakyIntegratorController li(K, alpha, dt);
// //     li.set_setpoint(Eigen::VectorXd::Zero(1));

// //     for (int i = 0; i < 10; ++i) {
// //         Eigen::VectorXd meas = Eigen::VectorXd::Constant(1, 1.0);
// //         Eigen::VectorXd out = li.process(meas);
// //         std::cout << "Step " << i << ", output = " << out.transpose() << "\n";
// //     }
// // }

// // // Stub test for Kalman until details are verified
// // void test_kalman() {
// //     std::cout << "--- Testing KalmanController (stub) ---\n";
// //     KalmanController kc(2, 1, 0.1);
// //     kc.set_setpoint(Eigen::VectorXd::Zero(1));

// //     for (int i = 0; i < 10; ++i) {
// //         Eigen::VectorXd meas = Eigen::VectorXd::Constant(1, 1.0);
// //         Eigen::VectorXd out = kc.process(meas);
// //         std::cout << "Step " << i << ", output = " << out.transpose() << "\n";
// //     }
// // }

// // int main() {
// //     test_pid();
// //     test_leaky();
// //     test_kalman();
// //     return 0;
// // }