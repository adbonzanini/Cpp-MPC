//
//  MPC.cpp
//  test
//
//  Created by Angelo Bonzanini on 12/7/20.
//

#include "MPC.hpp"
#include <cppad/cppad.hpp>
#include <Eigen/Core>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;      // number of predicted timesteps in the future
size_t nx = 6;      // 6 states
size_t nu = 2;      // 2 inputs
double dt = 0.12;   // system timestep
double ref_v = 100; // desired vehicle velocity

// The solver takes all the state variables and actuator
// variables in a single vector. Thus, we should to establish
// when one variable starts and another ends => define indices
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// Cost weights
double cte_penalty = 4500; // penalty for not having a low cross track error
double epsi_penalty = 8000; // for having an angle error
double speed_penalty = 3.25; // not following the speed limit
double steer_use_penalty = 200; // for steering the car
double a_use_penalty = 1; // using the throttle
double steer_change_penalty = 50; // having sharp / large steer angles between steps
double a_change_penalty = 1; // accelerating or braking fast

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;



class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs; //define vector that will be used to store the coefficients
    
    // CONSTRUCTOR
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
    
    // Define ADvector type
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    void operator()(ADvector & fg, const ADvector & vars){
        fg[0] = 0;
        
        // Penalize for path and/or speed deviation (states)
        for(int i=0; i<N; i++){
            // Indexing is first over the timesteps and then over the variables, i.e.
            // x1, x2, x3, ..., y1, y2, y3, ..., etc.
            fg[0]+=cte_penalty*CppAD::pow(vars[cte_start+i], 2);
            fg[0]+=epsi_penalty*CppAD::pow(vars[epsi_start+i], 2);
            fg[0]+=speed_penalty*CppAD::pow(vars[v_start+i]-ref_v, 2);
        }
        
        // Penalize steering and accelerating (inputs)
        for(int i=0; i<N-1; i++) {
            fg[0]+=steer_use_penalty*CppAD::pow(vars[delta_start+i], 2);
            fg[0]+=a_use_penalty*CppAD::pow(vars[a_start+i], 2);
        }
        
        // Penalize changes in steering and accelerating (change in inputs)
        for(int i=0; i<N-2; i++){
            fg[0]+=steer_change_penalty*CppAD::pow(vars[delta_start+i+1]-vars[delta_start+i], 2);
            fg[0]+=a_change_penalty*CppAD::pow(vars[a_start+i+1]-vars[a_start+i], 2);
        }
        
        // Initial constraints
        // Cost is at fg[0], so we need to add 1 to the indices (f[1] is the first constraint etc.)
        fg[1+x_start] = vars[x_start];
        fg[1+y_start] = vars[y_start];
        fg[1+psi_start] = vars[psi_start];
        fg[1+v_start] = vars[v_start];
        fg[1+cte_start] = vars[cte_start];
        fg[1+epsi_start] = vars[epsi_start];
        
        // Update the system constraints using the system model
        for(int i=0; i<N; i++){
            AD<double> x1 = vars[1+x_start+i];
            AD<double> y1 = vars[1+y_start+i];
            AD<double> psi1 = vars[1+psi_start+i];
            AD<double> v1 = vars[1+v_start+i];
            AD<double> cte1 = vars[1+cte_start+i];
            AD<double> epsi1 = vars[1+epsi_start+i];
            //AD<double> delta1 = vars[delta_start + i + 1];
            //AD<double> a1 = vars[a_start + i + 1];
            
            // Get the current timestep values
            AD<double> x0 = vars[x_start + i];
            AD<double> y0 = vars[y_start + i];
            AD<double> psi0 = vars[psi_start + i];
            AD<double> v0 = vars[v_start + i];
            //AD<double> cte0 = vars[cte_start + i];
            AD<double> epsi0 = vars[epsi_start + i];
            AD<double> delta0 = vars[delta_start + i];
            AD<double> a0 = vars[a_start + i];
            
            AD<double> fx0 = coeffs[0]+coeffs[1]*x0+coeffs[2]*x0*x0+coeffs[3]*x0*x0*x0;
            AD<double> psi_des = coeffs[1]+2*coeffs[2]*x0+3*coeffs[3]*x0*x0;
            
            // Set up the rest of the model constraints
            fg[2 + x_start+i] = x1 - (x0+v0*CppAD::cos(psi0)*dt);
            fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start + i] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
            fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + i] = cte1 - ((fx0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + i] = epsi1 - ((psi0 - psi_des) + (v0 / Lf * delta0 * dt));
        }
    }
};

// MPC class definition implementation --> constructor and destructor
MPC::MPC() {}
MPC::~MPC() {}

// Define Solve Method
std::tuple<vector<double>, vector<double>, vector<double>> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs){
    bool ok = true;
    
    // Define Dvector type (c.f. ADvector)
    typedef CPPAD_TESTVECTOR(double) Dvector;
    
    // Total number of variables
    size_t n_vars = N*nx + (N-1)*nu;
    size_t n_constraints = N*nx;
    
    // Initial value of the variables (states & inputs)
    Dvector vars(n_vars);
    for(int i=0; i<n_vars;i++){
        vars[i] = 0;
    }
    
    // Set lower and upper bounds for the variables
    Dvector vars_lowerBound(n_vars);
    Dvector vars_upperBound(n_vars);
    // Initialize inactive constraints
    for(int i=0; i<delta_start; i++){
        vars_lowerBound[i] = -1.0e19;
        vars_upperBound[i] = 1.0e19;
    }
    
    // Define input constraints
    for(int i=0; i<(N-1);i++){
        // Steer angle limits
        vars_lowerBound[delta_start+i] = -(25.0*M_PI/180)*Lf;
        vars_upperBound[delta_start+i] = (25.0*M_PI/180)*Lf;
        // Throttle limits
        vars_lowerBound[a_start+i] = -1.0;
        vars_upperBound[a_start+i] = 1.0;
    }
    
    // Define state constraints --> may be more complicated than just setting bounds to variables
    // Lower and upper limits for the constraints
    Dvector constraints_lowerBound(n_constraints);
    Dvector constraints_upperBound(n_constraints);
    for(int i=0; i<n_constraints; i++){
        constraints_lowerBound[i] = 0;
        constraints_upperBound[i]=0;
    }
    
    // state is an argument to the method
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];
    
    // Set lower bound = upper bound = initial state for the zeroth time step
    constraints_lowerBound[x_start] = x;
    constraints_lowerBound[y_start] = y;
    constraints_lowerBound[psi_start] = psi;
    constraints_lowerBound[v_start] = v;
    constraints_lowerBound[cte_start] = cte;
    constraints_lowerBound[epsi_start] = epsi;
    
    constraints_upperBound[x_start] = x;
    constraints_upperBound[y_start] = y;
    constraints_upperBound[psi_start] = psi;
    constraints_upperBound[v_start] = v;
    constraints_upperBound[cte_start] = cte;
    constraints_upperBound[epsi_start] = epsi;
    
    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    
    
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          1.0\n";
    
    
    // solve the problem
    CppAD::ipopt::solve_result<Dvector> solution;  // Define vector to store the solution

    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerBound, vars_upperBound, constraints_lowerBound, constraints_upperBound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    
    // Return the first actuator values. The variables can be accessed with
    vector<double> control = {solution.x[delta_start], solution.x[a_start]};
    vector<double> path_x;
    vector<double> path_y;
    for (int i=1; i < N-1; i++) {
        path_x.push_back(solution.x[x_start + i]);
        path_y.push_back(solution.x[y_start + i]);
    }
    return std::make_tuple(control, path_x, path_y);
}
