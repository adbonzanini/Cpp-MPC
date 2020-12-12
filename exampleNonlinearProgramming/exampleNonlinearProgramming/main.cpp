//
//  main.cpp
//  exampleNonlinearProgramming
//
//  Created by Angelo Bonzanini on 12/10/20.
//

#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include "buildProblem.hpp"

int main() {
    ProbStruct prob = defineProblem();
    SolStruct sol = solveProblem(prob);
    std::cout << sol.x << std::endl;
    std::cout << sol.status << std::endl;
    return 0;
}
