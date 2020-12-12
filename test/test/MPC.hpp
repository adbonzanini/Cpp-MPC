//
//  MPC.hpp
//  test
//
//  Created by Angelo Bonzanini on 12/7/20.
//

#ifndef MPC_hpp
#define MPC_hpp

#include <stdio.h>
#include <vector>
#include <Eigen/Core>
using namespace std;


class MPC {
public:
    MPC();
    virtual ~MPC();
    
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    std::tuple<vector<double>, vector<double>, vector<double>> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
};

#endif /* MPC_hpp */
