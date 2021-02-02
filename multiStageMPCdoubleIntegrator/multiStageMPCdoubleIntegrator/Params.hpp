//
//  Params.hpp
//  doubleIntegratorMPCclasses
//
//  Created by Angelo Bonzanini on 12/12/20.
//

#ifndef Params_hpp
#define Params_hpp

#include <stdio.h>
#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include <Eigen>
#include <random>
#include <cmath>
using namespace CppAD;

class Params{
public:
    // Variable size
    size_t nx = 2;
    const static size_t ny = 2;
    const static size_t nu = 1;
    const static size_t ng = 5;
    
    // Prediction horizon
    const static size_t N = 5;
    
    // Simulation horizon
    size_t Nsim = 10;
    
    // Scenario tree structure
    const static int NscNode = 2;                // Number of scenarios per node
    const static int Nrobust = 2;                // Robust horizon
    const static int Nsc = Nrobust*NscNode;      // Total number of scenarios
    const static int sumCommonNodes = 2+1+2;
    
    // Total number of variables
    size_t nVars = ((N+1)*nx+N*nu)*Nsc;
    size_t nConstraints = N*nx*Nsc+ng;
    
    // Start indices for variables
    size_t x1Start = 0;
    size_t x2Start = x1Start+N*Nsc;
    size_t u1Start = x2Start+N*Nsc;
    
    // State-space model for double integrator
    Eigen::Matrix<double, 2, 2> A, Areal;
    Eigen::Matrix<double, 2, 1> B, Breal;
    
    // Cost function matrices
    Eigen::Matrix<double, 2, 2> Q;
    double R;
    
    // Plant Noise
    Eigen::Matrix<double, 2, 20>  wNoise;
    
    // Noise bound for scenario tree and noise realization matrix
    double wB = 0.5;
    Eigen::Matrix<int, Nsc, N> wIdx;
    
    
    // Non-anticipativity constraint matrix
    const static int Erows = nu*sumCommonNodes;
    const static int Ecols = nu*N*Nsc;
    Eigen::Matrix<double, Erows, Ecols> Emat;
    
    
public:
    // Constructor
    Params();
};

#endif /* Params_hpp */
