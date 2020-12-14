//
//  FG_eval.hpp
//  doubleIntegrator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#ifndef FG_eval_hpp
#define FG_eval_hpp

#include <stdio.h>
#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include <Eigen>
using namespace CppAD;

struct Params{
    // Variable size
    size_t nx = 2;
    size_t nu = 1;
    size_t ng = 0;
    // Prediction horizon
    size_t N = 10;
    // Simulation horizon
    size_t Nsim = 20;
    // Total number of variables
    size_t nVars = (N+1)*nx+N*nu;
    size_t nConstraints = N*nx+ng;
    // Start indices for variables
    size_t x1Start = 0;
    size_t x2Start = x1Start+N;
    size_t u1Start = x2Start+N;
    // State-space model for double integrator
    Eigen::Matrix<double, 2, 2> A, C;
    Eigen::Matrix<double, 2, 1> B;
    // Cost function matrices
    Eigen::Matrix<double, 2, 2> Q;
    double R;
    // Constructor
    Params();
};

const Params p;

class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
//    ADvector fg, vars;
    void operator()(ADvector & fg, const ADvector & vars);
};
#endif /* FG_eval_hpp */
