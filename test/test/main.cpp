//
//  main.cpp
//  test
//
//  Created by Angelo Bonzanini on 12/7/20.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <cppad/cppad.hpp>
#include <math.h>
#include <chrono>
#include <thread>
#include <vector>
#include "MPC.hpp"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
            int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


//#################### MAIN ####################
int main(int argc, const char * argv[]) {
    
    // MPC is initialized here!
      MPC mpc;
    
    // Calculate steering angle and throttle using MPC.
    double target_psi = 0;
    double target_x = 0;
    double target_y = 0;
    
    // Calculate the cross track error
    double cte = 0;  // placeholder
    
    // Calculate the orientation error
    double epsi = 0; // placeholder
    
    // Create a vectors to store the states and inputs
    Eigen::VectorXd state(6);
    Eigen::VectorXd inputs(2);
    
    vector<double> control;
    vector<double> path_x;
    vector<double> path_y;
//    std::tie(control, path_x, path_y) = mpc.Solve(state, poly);
    
    return 0;
}
