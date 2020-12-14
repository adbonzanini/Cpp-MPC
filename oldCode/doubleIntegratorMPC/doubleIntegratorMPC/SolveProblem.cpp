//
//  solveProblem.cpp
//  doubleIntegratorExample
//
//  Created by Angelo Bonzanini on 12/10/20.
//

#include "SolveProblem.hpp"


SolveProblem::SolveProblem(){};

ReturnSol SolveProblem::solve(ReturnProb problem, Eigen::Vector2d initialState){
    
    // Update constraints according to the initial state
    problem.gLB[p.x1Start] = initialState[0];
    problem.gUB[p.x1Start] = initialState[0];
    problem.gLB[p.x2Start] = initialState[1];
    problem.gUB[p.x2Start] = initialState[1];
    
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(problem.options, problem.vars, problem.varsLB, problem.varsUB, problem.gLB, problem.gUB, problem.fg_eval, solution);
    
  
    /*
    if(solution.status == CppAD::ipopt::solve_result<Dvector>::success){
        std::cout<< "Successfully solved!" << std::endl;
    }
     */
    
    
    ReturnSol sol;
    sol.x =solution.x;
    sol.status = solution.status;
    
    return sol;
}
