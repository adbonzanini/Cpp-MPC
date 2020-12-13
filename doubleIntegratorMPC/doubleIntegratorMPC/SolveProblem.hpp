//
//  solveProblem.hpp
//  doubleIntegratorExample
//
//  Created by Angelo Bonzanini on 12/10/20.
//

#ifndef SolveProblem_hpp
#define SolveProblem_hpp

#include <stdio.h>
#include "BuildProblem.hpp"


class SolveProblem{
public:
    struct SolStruct{
        Dvector x, status;
    };
public:
    //Constructor
    SolveProblem();
    //Method
    ReturnSol solve(ReturnProb problem, Eigen::Vector2d initialState);
};


#endif /* SolveProblem_hpp */
