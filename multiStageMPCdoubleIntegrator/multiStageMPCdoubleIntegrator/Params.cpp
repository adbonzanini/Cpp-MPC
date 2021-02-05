//
//  Params.cpp
//  doubleIntegratorMPCclasses
//
//  Created by Angelo Bonzanini on 12/12/20.
//

#include "Params.hpp"

Params::Params(){
    
    // Initialize system matrices
    A <<    1, 1,
            0, 1;
    Areal = A;
    B <<    0.5,
            1;
    Breal = B;
    
    // Initialize cost matrices
    Q = Eigen::MatrixXd::Identity(2,2); Q(1,1) = 0.1;
    R = 0.1;
    
    // Initialize noise
    int seed = 0;
    std::default_random_engine generator (seed);
    std::normal_distribution<double> normDist (0.0,0.01);
    for(int k=0; k<Nsim; k++){wNoise(0,k) = normDist(generator); wNoise(1,k) = normDist(generator);}
    //*
    Emat << 1, 0, 0, 0, 0,   -1, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
            0, 1, 0, 0, 0,    0, -1, 0, 0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,    1, 0, 0, 0, 0,   -1, 0, 0, 0, 0,  0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,    0, 0, 0, 0, 0,   1, 0, 0, 0, 0,   -1, 0, 0, 0, 0,
            0, 0, 0, 0, 0,    0, 0, 0, 0, 0,   0, 1, 0, 0, 0,    0, -1, 0, 0, 0;
     //*/
    wIdx << 1,  1, 0, 0, 0,
            1, -1, 0, 0, 0,
            -1, 1, 0, 0, 0,
           -1, -1, 0, 0, 0;
            
}
