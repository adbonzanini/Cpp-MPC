//
//  buildProblem.cpp
//  exampleNonlinearProgramming
//
//  Created by Angelo Bonzanini on 12/10/20.
//

#include "buildProblem.hpp"

void FG_eval::operator()(ADvector & fg, const ADvector & x){
    
    // Objective function f(x)
    fg[0] = x[0]*x[3]*(x[0]+x[1]+x[2])+x[2];
    
    // Constraints
    // g_1(x)
    fg[1] = x[0] * x[1] * x[2] * x[3];
    // g_2(x)
    fg[2] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
    
    return;
}

//// Constructors
//SolStruct::SolStruct(Dvector x, Dvector status){
//    this->x = x;
//    this->status = status;
//}

//Params::Params(){
//    int abc = 5;
////    A << 1, 1,
////         0, 1;
////    B << 0.5, 0,
////         1, 0;
////    C = Eigen::MatrixXd::Identity(2,2);
//}


// Function that defines the problem
void build(){
        
    size_t i;
    
    // number of independent variables (domain dimension for f and g)
    size_t nx = 4;
    // number of constraints (range dimension for g)
    size_t ng = 2;
    
    // initial value of the independent variables
    Dvector xi(nx);
    xi[0] = 1.0;
    xi[1] = 5.0;
    xi[2] = 5.0;
    xi[3] = 1.0;
    
    // Lower and upper limits for x
    Dvector xl(nx), xu(nx);
    for(i=0; i<nx; i++){
        xl[i] = 1.0;
        xu[i] = 5.0;
    }
    
    // Lower and upper limits for g
    Dvector gl(ng), gu(ng);
    gl[0] = 25.0;
    gl[1] = 40.0;
    gu[0] = 1.0e19;
    gu[1] = 40.0;
    
    // Object that computes objective and constraints
    FG_eval fg_eval;
    
    // IPOPT options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     10\n";
    // approximate accuracy in first order necessary conditions;
    options += "Numeric tol          1e-6\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g., when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";
    
    // Alternative for initialization without a constructor
    ProbStruct problem = {xi, xl, xu, gl, gu, fg_eval, options};
    
    
    return;
}

