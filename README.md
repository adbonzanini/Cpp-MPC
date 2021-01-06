# Cpp-MPC
This folder contains some initial C++ projects built using Xcode in MacOS.

<h2> Required Libraries/packages </h2>

<ul>
  <li>CppAD: https://coin-or.github.io/CppAD/doc/cppad.htm. </br>
    In MacOS, you can install it using Homebrew by typing in the terminal <code>$ brew install cppad</code></li>
  <li>IPOPT: https://github.com/coin-or/Ipopt. </br>
    In MacOS, you can install it using Homebrew by typing in the terminal <code>$ brew install ipopt</code></li>
  <li>Eigen: https://gitlab.com/libeigen/eigen/-/releases/3.3.9. </br>
  Download the source code and link the .hpp and .cpp files </li>
  <li>Pbplots: https://github.com/InductiveComputerScience/pbPlots. </br>
  Clone the repository and link the .hpp and .cpp files </li>
  <li>Catch2: https://github.com/catchorg/Catch2.git. </br>
  In MacOS, you can install it using Homebrew by typing in the terminal <code>$ brew install catch2</code></li></li>
</ul>  


<h2> Projects </h2>
<ul>
  <li> <b>exampleNonlinearProgramming </b>: Solves a constrained optimization problem according to the CppAD documentation.</li>
 
  <li> <b>doubleIntegratorMPCclasses </b>: Solves the OCP for the double integrator in a receding horizon fashion to obtain the closed-loop solution. All of the MPC-related code is condensed into a single class called MPC.</li>
  
  <li> <b>doubleIntegratorMPCtests </b>: Same as doubleIntegratorMPCclasses, but also implements test cases using Catch 2. </li>
  
  <li> <b>oldCode </b>: Contains older Xcode projects that have been changed incrementally to produce the current versions. </br>
  doubleIntegratorOCP: Builds and solves the optimal control problem for a double integrator case study to obtain the open-loop solution. </br>
  doubleIntegratorMPC: Solves the OCP for the double integrator in a receding horizon fashion to obtain the closed-loop solution.</li>
</ul> 
