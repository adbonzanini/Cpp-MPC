//
//  CustomReturnTypes.hpp
//  doubleIntegrator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#ifndef CustomReturnTypes_hpp
#define CustomReturnTypes_hpp

#include <stdio.h>
#include "FG_eval.hpp"


// Define Dvector type
typedef CPPAD_TESTVECTOR( double ) Dvector;

// This is the struct that saves the formulated problem
struct ReturnProb{
    Dvector vars, varsLB, varsUB, gLB, gUB;
    std::string options;
    FG_eval fg_eval;
};

// This is the struct that saves the solution
struct ReturnSol{
    Dvector x, status;
};



#endif /* CustomReturnTypes_hpp */
