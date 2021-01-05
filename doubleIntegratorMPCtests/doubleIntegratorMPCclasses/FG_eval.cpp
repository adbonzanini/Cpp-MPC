//
//  FG_eval.cpp
//  doubleIntegrator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#include "FG_eval.hpp"


void FG_eval::operator()(ADvector & fg, const ADvector & vars){
    Params p;
    size_t j;
    // Define cost function (ignoring cross-interactions for now)
    fg[0] = 0;
    //*
    for(j=0; j<p.N-1; j++){
        fg[0] += p.Q(0,0)*CppAD::pow(vars[p.x1Start+j], 2);
        fg[0] += p.Q(1,1)*CppAD::pow(vars[p.x2Start+j], 2);
        fg[0] += p.R*CppAD::pow(vars[p.u1Start+j], 2);
    }
    
    fg[0] += p.Q(0,0)*CppAD::pow(vars[p.x1Start+p.N-1], 2);
    fg[0] += p.Q(1,1)*CppAD::pow(vars[p.x2Start+p.N-1], 2);
    
    // Initial Constraints
    // Add 1 to each of the starting indices due to cost being at index 0 of fg.
    fg[1+p.x1Start] = vars[p.x1Start];
    fg[1+p.x2Start] = vars[p.x2Start];
     
    // Update the system constraints using the systems model

    for(j=0; j<p.N-1;j++){
        // get the current timestep values => t
        AD<double> x1_0 = vars[p.x1Start + j];
        AD<double> x2_0 = vars[p.x2Start + j];
        AD<double> u1_0 = vars[p.u1Start + j];
        
        // get the future timestep values => t+1
        AD<double> x1_1 = vars[p.x1Start + j + 1];
        AD<double> x2_1 = vars[p.x2Start + j + 1];
        
        // Double integrator model
        AD<double> fx1 = p.A(0,0)*x1_0+p.A(0,1)*x2_0+p.B(0,0)*u1_0;
        AD<double> fx2 = p.A(1,0)*x1_0+p.A(1,1)*x2_0+p.B(1,0)*u1_0;
        
        // Constrain next state using the model
        fg[p.x1Start+j+2] = x1_1 - fx1;
        fg[p.x2Start+j+2] = x2_1 - fx2;
    }
    
    // Add nonlinear constraints (if any)
     for(j=0; j<p.ng;j++){
//        fg[p.u2Start+j+1] = 0;
    }
    // */
    return;
}
