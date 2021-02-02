//
//  FG_eval.cpp
//  doubleIntegrator
//
//  Created by Angelo Bonzanini on 02/02/21.
//

#include "FG_eval.hpp"


void FG_eval::operator()(ADvector & fg, const ADvector & vars){
    Params p;
    size_t j;

    // Define cost function (ignoring cross-interactions for now)
    fg[0] = 0;
    double wSc = 1./p.Nsc;
    //*
    for(j=0; j<p.N-1; j++){
        for(int s=0; s<p.Nsc; s++){
            fg[0] += wSc*p.Q(0,0)*CppAD::pow(vars[p.x1Start+j]-10.0, 2);
            fg[0] += wSc*p.Q(1,1)*CppAD::pow(vars[p.x2Start+j], 2);
            fg[0] += wSc*p.R*CppAD::pow(vars[p.u1Start+j], 2);
        }
    }
    
    fg[0] += wSc*p.Q(0,0)*CppAD::pow(vars[p.x1Start+p.N-1], 2);
    fg[0] += wSc*p.Q(1,1)*CppAD::pow(vars[p.x2Start+p.N-1], 2);
    
    // Initial Constraints
    // Add 1 to each of the starting indices due to cost being at index 0 of fg.
    fg[1+p.x1Start] = vars[p.x1Start];
    fg[1+p.x2Start] = vars[p.x2Start];

    
    // Update the system constraints using the systems model

    for(int s=0;s<p.Nsc; s++){
        for(j=0; j<p.N-1; j++){
            // get the current timestep values => t
            AD<double> x1_0 = vars[p.x1Start + j + s];
            AD<double> x2_0 = vars[p.x2Start + j + s];
            AD<double> u1_0 = vars[p.u1Start + j + s];
            
            // get the future timestep values => t+1
            AD<double> x1_1 = vars[p.x1Start + j+1 + s];
            AD<double> x2_1 = vars[p.x2Start + j+1 + s];
            
            // Double integrator model
            AD<double> fx1 = p.A(0,0)*x1_0+p.A(0,1)*x2_0+p.B(0,0)*u1_0 + p.wIdx(s,j)*p.wB;
            AD<double> fx2 = p.A(1,0)*x1_0+p.A(1,1)*x2_0+p.B(1,0)*u1_0 + p.wIdx(s,j)*p.wB;
            
            // Constrain next state using the model
            fg[2+p.x1Start+j+s] = x1_1 - fx1;
            fg[2+p.x2Start+j+s] = x2_1 - fx2;
        }
    }
    
    // Addiional constraints (non-anticipativity)
    if(p.ng>0){
        j = 0; int s1 = 1; int s2 = 2;
        fg[1+0+p.u1Start] = vars[p.u1Start+j+s1-1]-vars[p.u1Start+j+p.N*(s2-1)];
        
        j = 1; s1 = 1; s2 = 2;
        fg[1+1+p.u1Start] = vars[p.u1Start+j+s1-1]-vars[p.u1Start+j+p.N*(s2-1)];
        
        j = 0; s1 = 2; s2 = 3;
        fg[1+2+p.u1Start] = vars[p.u1Start+j+s1-1]-vars[p.u1Start+j+p.N*(s2-1)];
        
        j = 0; s1 = 3; s2 = 4;
        fg[1+3+p.u1Start] = vars[p.u1Start+j+s1-1]-vars[p.u1Start+j+p.N*(s2-1)];
        
        j = 1; s1 = 3; s2 = 4;
        fg[1+4+p.u1Start] = vars[p.u1Start+j+s1-1]-vars[p.u1Start+j+s2-1];
    }
    std::cout<< fg.size() << std::endl;
    
    return;
}
