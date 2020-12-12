//
//  main.cpp
//  doubIntegratator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#include <iostream>
#include "SolveProblem.hpp"


int main(int argc, const char * argv[]) {
    BuildProblem problem;
    SolveProblem solution;
    
    Eigen::Vector2d initialState;
    initialState(0) = 2; initialState(1) = 1;
    ReturnSol sol = solution.solve(problem.build(), initialState);
    
    std::cout << sol.x << std::endl;


}
