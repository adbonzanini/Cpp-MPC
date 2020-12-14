//
//  MPC.hpp
//  doubleIntegratorMPCclasses
//
//  Created by Angelo Bonzanini on 12/12/20.
//

#ifndef MPC_hpp
#define MPC_hpp

#include <stdio.h>
#include "FG_eval.hpp"

// Define Dvector type
typedef CPPAD_TESTVECTOR( double ) Dvector;

class MPC{
    
public:
    
    
    // Define the struct that saves the formulated problem
    struct ReturnProb{
        Dvector vars, varsLB, varsUB, gLB, gUB;
        std::string options;
        FG_eval fg_eval;
    };
    
    // Define the struct that saves the solution of the OCP
    struct ReturnSol{
        Dvector x, status;
    };
    
    struct ReturnTraj{
        double uOpt;
        std::vector<double> x1Path;
        std::vector<double> x2Path;
        std::vector<double> u1Path;
        // Constructor
        ReturnTraj();
    };
    
public:
    
    //Constructor
    MPC();
    
    //Methods
    ReturnProb ocpBuild(Params p);
    ReturnSol ocpSolve(Params p, ReturnProb problem, Eigen::Vector2d initialState);
    ReturnTraj mpcSolve(Params p, ReturnProb problem, Eigen::VectorXd initialState);
};



#endif /* MPC_hpp */


