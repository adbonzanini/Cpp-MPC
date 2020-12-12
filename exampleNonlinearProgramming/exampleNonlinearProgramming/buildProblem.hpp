//
//  buildProblem.hpp
//  exampleNonlinearProgramming
//
//  Created by Angelo Bonzanini on 12/10/20.
//

#ifndef buildProblem_hpp
#define buildProblem_hpp

#include <stdio.h>
#include <iostream>
#include <cppad/ipopt/solve.hpp>
using namespace CppAD;

typedef CPPAD_TESTVECTOR( double ) Dvector;

class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector & fg, const ADvector & x);
};

struct SolStruct{
    Dvector x, status;
    //Constructor
    SolStruct(Dvector x, Dvector status);
};

struct ProbStruct{
    Dvector xi, xl, xu, gl, gu;
    FG_eval fg_eval;
    std::string options;
};

ProbStruct defineProblem();
SolStruct solveProblem(ProbStruct problem);



#endif /* buildProblem_hpp */
