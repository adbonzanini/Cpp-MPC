//
//  buildProblem.cpp
//  exampleNonlinearProgramming
//
//  Created by Angelo Bonzanini on 12/10/20.
//

#include "BuildProblem.hpp"

BuildProblem::BuildProblem(){};

ReturnProb BuildProblem::build(){

    size_t j;
    size_t nConstraints = p.nConstraints;
    

    // initial value of the variables to zero
    Dvector vars(p.nVars);
    for(j=0;j<p.nVars;j++){
        vars[j] = 0.0;
    }
    
    // Lower and upper limits for vars
    Dvector varsLB(p.nVars), varsUB(p.nVars);
    for(j=0; j<p.nVars; j++){
        varsLB[j] = -10;
        varsUB[j] = 10;
    }
    
    // Lower and upper limits for g
    Dvector gLB(nConstraints), gUB(nConstraints);
    for(j=0;j<nConstraints;j++){
        gLB[j] = 0;
        gUB[j] = 0;
    }
    
    
    // Object that computes objective and constraints
    FG_eval fg_eval;
    
    // IPOPT options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     100\n";
    // approximate accuracy in first order necessary conditions;
    options += "Numeric tol          1e-6\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g., when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";
    
    // Alternative for initialization without a constructor
    ReturnProb problem;
    problem.vars = vars;
    problem.varsLB = varsLB;
    problem.varsUB = varsUB;
    problem.gLB = gLB;
    problem.gUB = gUB;
    problem.fg_eval = fg_eval;
    problem.options = options;
    
    
    return problem;
}
