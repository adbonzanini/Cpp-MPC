//
//  Tests.hpp
//  doubleIntegratorMPCclasses
//
//  Created by Angelo Bonzanini on 1/5/21.
//

#ifndef Tests_hpp
#define Tests_hpp

#include <catch2/catch.hpp>
#include <stdio.h>
#include "MPC.hpp"

MPC::ReturnTraj solveMPC(Params p, Eigen::Vector2d x0);

#endif /* Tests_hpp */
