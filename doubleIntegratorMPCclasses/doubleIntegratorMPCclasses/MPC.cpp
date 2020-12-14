//
//  MPC.cpp
//  doubleIntegratorMPCclasses
//
//  Created by Angelo Bonzanini on 12/12/20.
//

#include "MPC.hpp"

// Constructors
MPC::MPC(){};

MPC::ReturnTraj::ReturnTraj(){
}


// Methods
MPC::ReturnProb MPC::ocpBuild(Params p){

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
    MPC::ReturnProb problem;
    problem.vars = vars;
    problem.varsLB = varsLB;
    problem.varsUB = varsUB;
    problem.gLB = gLB;
    problem.gUB = gUB;
    problem.fg_eval = fg_eval;
    problem.options = options;
    
    
    return problem;
}



MPC::ReturnSol MPC::ocpSolve(Params p, ReturnProb problem, Eigen::Vector2d initialState){
    
    // Update constraints according to the initial state
    problem.gLB[p.x1Start] = initialState[0];
    problem.gUB[p.x1Start] = initialState[0];
    problem.gLB[p.x2Start] = initialState[1];
    problem.gUB[p.x2Start] = initialState[1];
    
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(problem.options, problem.vars, problem.varsLB, problem.varsUB, problem.gLB, problem.gUB, problem.fg_eval, solution);
    
    MPC::ReturnSol sol;
    sol.x =solution.x;
    sol.status = solution.status;
    
    return sol;
}




MPC::ReturnTraj MPC::mpcSolve(Params p, ReturnProb problem, Eigen::VectorXd x0){
    
    // Define variables to save the MPC solution
    double uOpt;
    std::vector<double> x1Path(p.Nsim+1);
    std::vector<double> x2Path(p.Nsim+1);
    std::vector<double> u1Path(p.Nsim);
    
    // Define initial state
    x1Path[0] = x0[0];
    x2Path[0] = x0[1];
    
    
    // Loop
    for(int k=0; k<p.Nsim;k++){
        
        // Solve OCP
        MPC::ReturnSol sol = ocpSolve(p, problem, x0);
        // Extract optimal input
        uOpt = sol.x[p.u1Start];
        u1Path[k] = uOpt;
        
        // Apply optimal input to the system
        x0 = p.A*x0 + p.B*uOpt+ (p.wNoise).col(k);
        x1Path[k+1] = x0[0]; x2Path[k+1] = x0[1];
        
    }
    
    // Define return struct
    ReturnTraj tr;
    tr.x1Path = x1Path;
    tr.x2Path = x2Path;
    tr.u1Path = u1Path;
    
    return tr;
}
