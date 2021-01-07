//
//  Tests.cpp
//  doubleIntegratorMPCclasses
//
//  Created by Angelo Bonzanini on 1/5/21.
//

#include "Tests.hpp"

//***********
// FUNCTIONS
//***********

// Function that solves the MPC problem
MPC::ReturnTraj solveMPC(Params p, Eigen::Vector2d x0){
    
    // Create optimal control problem through the MPC class
    MPC mpc;
    MPC::ReturnProb ocp = mpc.ocpBuild(p);
    
    // Solve the MPC
    MPC::ReturnTraj tr = mpc.mpcSolve(p, ocp, x0);
    
    return tr;
}

// Function that returns the last nElements of a trajectory vector
std::vector<double> getLastElements(std::vector<double> vec, Params p, int nElements){
    
    std::vector<double> returnVec = std::vector<double>(vec.begin()+p.Nsim-1-nElements, vec.end());
    
    return returnVec;
}


//***********
// TESTS
//***********

// Ensure that the trajectories converge in a neigborhood around the origin
TEST_CASE("Convergence to Setpoint"){
    
    // Define constants
    const double SETPOINT = 0;      // Setpoint around which the trajectories should converge
    const double TOL = 0.1;         // Tolerance for approximate equality due to noise
    const int NSTARTINGPOINTS = 1;  // How many starting points (for each state) to test randomly
    srand (1);                      // Seed
    
    // Define parameters and initial conditions
    Params p;
    Eigen::VectorXd x0(p.nx);
    x0(0) = GENERATE(take(NSTARTINGPOINTS, random(-10, 10)));
    x0(1) = GENERATE(take(NSTARTINGPOINTS, random(-10, 10)));
    // Solve MPC
    MPC::ReturnTraj tr = solveMPC(p,x0);
    
    // Obtain the last n elements of the trajectories
    std::vector<double> x1end = getLastElements(tr.x1Path, p, 5);
    std::vector<double> x2end = getLastElements(tr.x2Path, p, 5);
    std::vector<double> u1end = getLastElements(tr.u1Path, p, 5);
    
    // Define the expected values
    std::vector<double> x1Expect(x1end.size(), SETPOINT);
    std::vector<double> x2Expect(x2end.size(), SETPOINT);
    std::vector<double> u1Expect(u1end.size(), SETPOINT);
    
    // Check approximate equality (due to noise)
    // Use REQUIRE_THAT macro to use matchers (used for more complex comparisons)
    REQUIRE_THAT(x1end, Catch::Approx(x1Expect).margin(TOL));
    REQUIRE_THAT(x2end, Catch::Approx(x2Expect).margin(TOL));
    REQUIRE_THAT(u1end, Catch::Approx(u1Expect).margin(TOL));
    

}


//EXAMPLES OF TEST CASES
/*
unsigned int Factorial( unsigned int number ) {
    return number > 1 ? Factorial(number-1)*number : 1;
}


TEST_CASE("The ultimate answer"){
    REQUIRE( 6*7 == 42);
}

TEST_CASE("10x10 ints"){
    auto x = GENERATE(range(1, 11));
    auto y = GENERATE(range(101, 111));
    
    CHECK(x<y);
}

TEST_CASE("Random numbers in a range", "[.] [approvals]"){
    auto x = GENERATE( -1000+rand()%10000+1 );
    auto y = GENERATE(random(-1000, 1000));
    CAPTURE(x);
    REQUIRE(  x*x>= 0);
    REQUIRE(  y*y>= 0);
}

TEST_CASE( "Factorials are computed", "[factorial]" ) {
    CHECK( Factorial(0) == 1 );
    CHECK( Factorial(1) == 1 );
    CHECK( Factorial(2) == 2 );
    CHECK( Factorial(3) == 6 );
    CHECK( Factorial(10) == 3628800 );
}

TEST_CASE("Vectors can be sized and resized", "[vector]"){
    std::vector<int> v(5);
    
    REQUIRE(v.size()==5);
    REQUIRE(v.capacity()>=5);
    
    // Each section executes the test case from the start!
    SECTION("Resizing bigger changes size and capacity"){
        v.resize(10);
        
        REQUIRE(v.size()==10);
        REQUIRE(v.capacity()>=10);
    }
    
    SECTION("Resizing smaller changes size but not capacity"){
        v.resize(1);
        
        REQUIRE(v.size()==1);
        REQUIRE(v.capacity()>=5);
    }
    
    SECTION("reserving bigger changes capacity but not size"){
        v.reserve(10);
        
        REQUIRE(v.size()==5);
        REQUIRE(v.capacity()>=10);
        
        SECTION("reserving smaller again does not change capacity"){
            v.reserve(7);
            REQUIRE(v.capacity()>=10);
        }
    }
        
    
    SECTION("reserving smaller does not change capacity or size"){
        v.reserve(1);
        
        REQUIRE(v.size()==5);
        REQUIRE(v.capacity()>=5);
    }
    
}
*/
