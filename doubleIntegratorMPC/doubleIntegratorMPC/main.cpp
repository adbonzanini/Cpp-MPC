//
//  main.cpp
//  doubIntegratator
//
//  Created by Angelo Bonzanini on 12/11/20.
//

#include <iostream>
#include <math.h>
#include <chrono>
#include <random>
#include "pbPlots.hpp"
#include "supportLib.hpp"
#include "SolveProblem.hpp"



int main(int argc, const char * argv[]) {
    
    // Create BuildProblem and SolveProblem objects
    BuildProblem problem;
    SolveProblem solution;
    
    // Define random noise
    int seed = 0;
    std::default_random_engine generator (seed);
    std::normal_distribution<double> normDist (0.0,0.01);
    Eigen::MatrixXd wNoise(2, p.Nsim);
    for(int k=0; k<p.Nsim; k++){wNoise(0,k) = normDist(generator); wNoise(1,k) = normDist(generator);}
    
    // Define variables to save the MPC solution
    Eigen::VectorXd uOpt(p.nu);
    std::vector<double> x1Path(p.Nsim+1);
    std::vector<double> x2Path(p.Nsim+1);
    std::vector<double> u1Path(p.Nsim);
    
    // Define initial state
    Eigen::VectorXd x0(p.nx); x0(0) = 5; x0(1) = 5;
    x1Path[0] = x0[0]; x2Path[0] = x0[1];
    
    
    // Loop
    auto start = std::chrono::high_resolution_clock::now();
    for(int k=0; k<p.Nsim;k++){
        // Solve OCP
        ReturnSol sol = solution.solve(problem.build(), x0);
        // Extract optimal input
        uOpt[0] = sol.x[p.u1Start];
        u1Path[k] = uOpt[0];
        
        // Apply optimal input to the system
        x0 = p.A*x0 + p.B*uOpt+ wNoise.col(k);
        x1Path[k+1] = x0[0]; x2Path[k+1] = x0[1];
        
    }
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    
    // To add a value: xPlot.push_back(value)
    
    // Print solution
    std::cout << "x1 = ";
    for(int j=0;j<p.N;j++){std::cout << x1Path[j] << " ";} std::cout << std::endl;
    std::cout << "x2 = ";
    for(int j=0;j<p.N;j++){std::cout << x2Path[j] << " ";} std::cout << std::endl;
    std::cout << "u1 = ";
    for(int j=0;j<p.N;j++){std::cout << u1Path[j] << " ";} std::cout << std::endl;
    
    
    start = std::chrono::high_resolution_clock::now();
    
    
    // Create discrete time vectors
    std::vector<double> discreteTime(p.Nsim+1);
    std::iota(discreteTime.begin(), discreteTime.end(), 0);
    std::vector<double> discreteTimeU = std::vector<double> (discreteTime.begin(), discreteTime.end()-1);
    // Print last element
//    std::cout << discreteTime.back() << " " << discreteTimeU.back() << std::endl;
    
    
    // Plotting and saving
    RGBABitmapImageReference *imageRef = CreateRGBABitmapImageReference();
    string imgLoc = "/Users/adbonzanini/Box Sync/Berkeley/Research/Cpp-MPC/doubleIntegratorMPCclasses/doubleIntegratorMPCclasses";
    string imgTitle;
    std::vector<double> *pngData;
    
    // Plot 1
    imgTitle = "phasePlot.png";
    DrawScatterPlot(imageRef, 600, 400, &x1Path, &x2Path);
    pngData = ConvertToPNG(imageRef->image);
    WriteToFile(pngData, imgLoc+imgTitle);
    
    
    // Plot 2
    imgTitle = "x1.png";
    DrawScatterPlot(imageRef, 600, 400, &discreteTime, &x1Path);
    pngData = ConvertToPNG(imageRef->image);
    WriteToFile(pngData, imgLoc+imgTitle);
    
    // Plot 3
    imgTitle = "x2.png";
    DrawScatterPlot(imageRef, 600, 400, &discreteTime, &x2Path);
    pngData = ConvertToPNG(imageRef->image);
    WriteToFile(pngData, imgLoc+imgTitle);

    
    DeleteImage(imageRef->image);
    
    stop = std::chrono::high_resolution_clock::now();
    std::cout << std::endl;
    std::cout << "Avg. time per iteration = " <<duration.count()/p.Nsim << " ms" << std::endl;
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time for plot and save  = " << duration.count() << " ms"  << std::endl;
     

    return 0;
}
